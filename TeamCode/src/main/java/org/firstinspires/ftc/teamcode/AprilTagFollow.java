package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * AprilTag Follow: the robot locks onto ONE AprilTag and keeps itself
 * TARGET_DISTANCE inches straight in front of it, square to the tag's face.
 * Move the tag and the robot follows ("dances") with it.
 *
 * CONTROLS (gamepad 1):
 *   HOLD left bumper  -> follow the tag. Let go -> robot stops instantly.
 *   dpad up / down    -> raise / lower the top speed (shown on the Driver Station)
 *
 * SAFETY: it only moves while the left bumper is held. Let go -> stops, always.
 *
 * IF IT LOSES THE TAG (bumper still held):
 *   1. for LOST_COAST_SECONDS it keeps doing what it was doing, fading out --
 *      usually the tag shows up again (it was just blurry, or slipped out of view)
 *   2. then, if the tag was last seen off to one side, it turns that way for
 *      up to SEARCH_SECONDS to find it again
 *   3. then it gives up and stops
 *
 * Based on the FTC SDK sample RobotAutoDriveToAprilTagOmni. Each loop it
 * measures three "errors" and fixes each one with a different wheel motion:
 *   - too far / too close           -> drive forward / back
 *   - tag off to one side of camera -> turn to point at it
 *   - not square to the tag's face  -> strafe sideways around it
 * Each push = MIN_POWER + error x GAIN. MIN_POWER is there because wheels
 * don't move at all below some power (friction), so a tiny push does nothing.
 */
@TeleOp(name = "AprilTag Follow", group = "Test")
public class AprilTagFollow extends LinearOpMode {

    // ---------------- WHAT TO FOLLOW ----------------
    private static final int    TARGET_TAG_ID   = 30;   // any other tag is ignored
    private static final double TARGET_DISTANCE = 36.0; // inches, camera to tag, along the floor

    // ---------------- WHERE THE CAMERA IS ----------------
    // true = camera is on the BACK of the robot, looking backward.
    // Then "get closer to the tag" means driving BACKWARD, and the camera's
    // left is the robot's right, so drive and strafe flip. Turning does NOT
    // flip: spinning the robot left spins the camera left too, either way.
    private static final boolean CAMERA_FACES_BACKWARD = true;

    // Measured on the robot. Centered left/right.
    private static final double CAMERA_HEIGHT    = 18.0; // inches above the floor
    // How far the camera is tipped UP (0 = level). On the field the tags are on
    // the BOTTOM of the hive cells, so a hive camera will probably tip up.
    // Change this when it does -- the distance math below already uses it.
    private static final double CAMERA_PITCH_DEG = 0.0;

    // ---------------- HOW HARD TO PUSH ----------------
    // GAIN = extra power per inch (or degree) of error. If the robot wobbles
    // back and forth around the target, lower it. If it's sluggish, raise it.
    private static final double DRIVE_GAIN  = 0.025; // per inch of distance error
    private static final double STRAFE_GAIN = 0.02;  // per degree of "not square"
    private static final double TURN_GAIN   = 0.012; // per degree of "not pointed at it"

    // MIN_POWER = the smallest push that actually moves the robot. To tune: if
    // the screen shows a push but the robot doesn't move, raise it. If the robot
    // jumps past the target and back, lower it. Strafing needs the most.
    private static final double MIN_DRIVE_POWER  = 0.08;
    private static final double MIN_STRAFE_POWER = 0.15;
    private static final double MIN_TURN_POWER   = 0.08;

    // "Close enough": inside these, don't push at all. The camera's readings
    // jump around a few degrees even when nothing moves, so these must be
    // bigger than that jumping or the robot chases noise. Yaw is the jumpiest.
    private static final double DISTANCE_TOLERANCE = 2.0; // inches
    private static final double YAW_TOLERANCE      = 8.0; // degrees
    private static final double BEARING_TOLERANCE  = 5.0; // degrees

    // ---------------- SMOOTHING ----------------
    // Average the readings so one bad camera frame can't cause a kick.
    // How much each NEW frame counts: 1.0 = no smoothing (jumpy but quick),
    // 0.1 = very smooth (calm but slow to notice the tag moved).
    private static final double SMOOTHING = 0.3;

    // ---------------- WHEN THE TAG IS LOST ----------------
    // Keep going this long, fading to a stop. Keep it SHORT: the robot is
    // driving blind, and too long makes it overshoot and wobble.
    private static final double LOST_COAST_SECONDS = 0.2;
    private static final double SEARCH_SECONDS     = 3.0; // then turn to look this long
    // Only search if the tag was last seen at least this far to one side.
    // (If it was straight ahead it probably went too far away -- turning won't help.)
    private static final double SEARCH_MIN_BEARING = 10.0; // degrees

    // ---------------- CAMERA PICTURE ----------------
    // Short exposure = less blur when moving, but a darker picture. If the
    // Camera Stream looks dark and it loses the tag even when still, raise
    // EXPOSURE_MS (try 10-15). If it loses the tag only when moving, lower it.
    private static final int EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;

    // ---------------- SPEED LIMIT (dpad up/down) ----------------
    private static final double START_SPEED = 0.25; // start slow
    private static final double SPEED_STEP  = 0.05;
    private static final double MIN_SPEED   = 0.05;
    private static final double MAX_SPEED   = 0.60; // dpad can't go past this
    // Turning is touchier than driving, so it gets a smaller share of the limit
    private static final double TURN_SHARE  = 0.6;

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double maxSpeed = START_SPEED;

    @Override
    public void runOpMode() {
        initDriveMotors();
        initCamera();

        telemetry.addLine("Follows tag " + TARGET_TAG_ID + " at " + TARGET_DISTANCE + " in.");
        telemetry.addLine("Camera faces " + (CAMERA_FACES_BACKWARD ? "BACKWARD" : "forward"));
        telemetry.addLine("HOLD left bumper to follow. dpad up/down = speed.");
        telemetry.addLine("Camera view: 3 dots menu > Camera Stream");
        telemetry.update();
        waitForStart();

        ElapsedTime sinceLastSeen = new ElapsedTime();
        boolean everSeen = false;
        double lastBearing = 0;                          // where the tag was when last seen
        double lastDrive = 0, lastStrafe = 0, lastTurn = 0; // what we were doing then

        // Smoothed (averaged) readings
        boolean smoothReady = false;
        double smoothDistance = 0, smoothBearing = 0, smoothYaw = 0;
        long lastFrameTime = 0; // to count each camera picture only once

        while (opModeIsActive()) {
            // Speed limit: one step per press
            if (gamepad1.dpadUpWasPressed())   maxSpeed += SPEED_STEP;
            if (gamepad1.dpadDownWasPressed()) maxSpeed -= SPEED_STEP;
            maxSpeed = Range.clip(maxSpeed, MIN_SPEED, MAX_SPEED);

            boolean following = gamepad1.left_bumper;
            AprilTagDetection tag = findTargetTag();

            double drive = 0, strafe = 0, turn = 0;
            double lostFor = sinceLastSeen.seconds();

            if (tag == null) {
                if (!following || !everSeen) {
                    telemetry.addLine("Tag " + TARGET_TAG_ID + ": NOT SEEN -> stopped");
                } else if (lostFor < LOST_COAST_SECONDS) {
                    // Probably just a blurry frame or two: keep going, fading out
                    double fade = 1.0 - lostFor / LOST_COAST_SECONDS;
                    drive = lastDrive * fade; strafe = lastStrafe * fade; turn = lastTurn * fade;
                    telemetry.addLine(String.format("Tag LOST %.1fs -> coasting", lostFor));
                } else if (lostFor < LOST_COAST_SECONDS + SEARCH_SECONDS
                        && Math.abs(lastBearing) >= SEARCH_MIN_BEARING) {
                    // It went out the side of the camera's view: turn that way
                    turn = Math.signum(lastBearing) * maxSpeed * TURN_SHARE;
                    telemetry.addLine(String.format("Tag LOST %.1fs -> searching %s",
                            lostFor, lastBearing > 0 ? "LEFT" : "RIGHT"));
                } else {
                    telemetry.addLine("Tag LOST -> gave up, stopped");
                }
            } else {
                sinceLastSeen.reset();
                everSeen = true;
                TagPosition pos = whereIsTheTag(tag);
                double rawYaw = tag.ftcPose.yaw;

                // Smooth the readings. Start fresh if the tag was really gone
                // (old readings are wrong by now). Only count NEW camera
                // pictures -- this loop runs faster than the camera.
                boolean newFrame = tag.frameAcquisitionNanoTime != lastFrameTime;
                lastFrameTime = tag.frameAcquisitionNanoTime;
                if (!smoothReady || lostFor > LOST_COAST_SECONDS) {
                    smoothDistance = pos.forward;
                    smoothBearing  = pos.bearing;
                    smoothYaw      = rawYaw;
                    smoothReady = true;
                } else if (newFrame) {
                    smoothDistance += SMOOTHING * (pos.forward - smoothDistance);
                    smoothBearing  += SMOOTHING * (pos.bearing - smoothBearing);
                    smoothYaw      += SMOOTHING * (rawYaw      - smoothYaw);
                }

                double distanceError = smoothDistance - TARGET_DISTANCE; // + = too far away
                double bearingError  = smoothBearing;                    // + = tag is to our left
                double yawError      = smoothYaw;                        // + = we're off to one side of its face
                lastBearing = bearingError;

                if (following) {
                    double turnLimit = maxSpeed * TURN_SHARE;
                    drive  = push(distanceError, DRIVE_GAIN,  MIN_DRIVE_POWER,  DISTANCE_TOLERANCE, maxSpeed);
                    strafe = push(-yawError,     STRAFE_GAIN, MIN_STRAFE_POWER, YAW_TOLERANCE,      maxSpeed);
                    turn   = push(bearingError,  TURN_GAIN,   MIN_TURN_POWER,   BEARING_TOLERANCE,  turnLimit);
                }

                telemetry.addLine("Tag " + TARGET_TAG_ID + ": SEEN"
                        + (following ? " -> FOLLOWING" : " (hold left bumper)"));
                // "smooth" is what the robot acts on; "raw" is this one frame.
                // Watch raw jump around with the tag still -- that's the noise.
                telemetry.addData("Distance (in)", "%5.1f  raw %5.1f  (want %.0f)",
                        smoothDistance, pos.forward, TARGET_DISTANCE);
                telemetry.addData("Bearing (deg)", "%+5.1f  raw %+5.1f  (want 0 +/-%.0f, + = left)",
                        bearingError, pos.bearing, BEARING_TOLERANCE);
                telemetry.addData("Yaw (deg)", "%+5.1f  raw %+5.1f  (want 0 +/-%.0f)",
                        yawError, rawYaw, YAW_TOLERANCE);
                telemetry.addData("Sideways (in)", "%+5.1f  (+ = to the CAMERA's right)", pos.right);
                telemetry.addData("Tag height (in)", "%5.1f  (above the floor)", pos.height);
                telemetry.addData("Straight-line range (in)", "%5.1f", tag.ftcPose.range);

                // Remember what we're doing, in case the next frames lose the tag
                lastDrive = drive; lastStrafe = strafe; lastTurn = turn;
            }

            // Everything above is worked out from the CAMERA's point of view.
            // If the camera looks out the back, flip drive and strafe so the
            // robot moves the right way. (Turn stays the same -- see the top.)
            if (CAMERA_FACES_BACKWARD) {
                drive = -drive;
                strafe = -strafe;
            }

            // The bumper always wins: not held = not moving, whatever happened above
            if (!following) {
                drive = 0; strafe = 0; turn = 0;
            }
            moveRobot(drive, strafe, turn);

            telemetry.addData("Top speed", "%.2f  (dpad up/down)", maxSpeed);
            telemetry.addData("Wheels (robot)", "drive %+.2f  strafe %+.2f  turn %+.2f", drive, strafe, turn);
            telemetry.update();
        }

        moveRobot(0, 0, 0);
        visionPortal.close();
    }

    // How hard to push for one error: nothing if we're close enough, otherwise
    // at least minPower (so the wheels actually move) plus more for bigger
    // errors, never more than limit. The sign of the error picks the direction.
    private static double push(double error, double gain, double minPower,
                               double tolerance, double limit) {
        if (Math.abs(error) <= tolerance) return 0;
        double power = Math.signum(error) * (minPower + Math.abs(error) * gain);
        return Range.clip(power, -limit, limit);
    }

    // Tag 30 if the camera sees it, otherwise null. Other tags are ignored.
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            // metadata == null means it's not a BIOBUZZ tag, so we can't measure it
            if (detection.id == TARGET_TAG_ID && detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    // Where the tag is compared to the ROBOT, not the camera.
    // The camera reports: x = right, y = straight out of the lens, z = up (from
    // the lens's point of view). If the camera is tipped up, "out of the lens"
    // is partly "up", so we un-tip it to get distance along the floor.
    // With CAMERA_PITCH_DEG = 0 this changes nothing.
    private static class TagPosition {
        double forward; // inches, along the floor, straight ahead of the camera
        double right;   // inches, + = to the right
        double height;  // inches above the floor
        double bearing; // degrees, + = to the LEFT (same as the SDK)
    }

    private TagPosition whereIsTheTag(AprilTagDetection tag) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double x = tag.ftcPose.x, y = tag.ftcPose.y, z = tag.ftcPose.z;

        TagPosition pos = new TagPosition();
        pos.forward = y * Math.cos(pitch) - z * Math.sin(pitch);
        pos.right   = x;
        pos.height  = CAMERA_HEIGHT + y * Math.sin(pitch) + z * Math.cos(pitch);
        pos.bearing = Math.toDegrees(Math.atan2(-pos.right, pos.forward));
        return pos;
        // NOTE for the hive: a tag on the bottom of a cell lies FLAT, so its
        // "yaw" means something different than for a tag standing up. When we
        // aim at the hive, the strafe part will need to change. Distance and
        // bearing above already work for a flat tag.
    }

    // Same mecanum math as the SDK sample.
    // drive + = forward, strafe + = left, turn + = counter-clockwise (left)
    private void moveRobot(double drive, double strafe, double turn) {
        double lf = drive - strafe - turn;
        double rf = drive + strafe + turn;
        double lr = drive + strafe - turn;
        double rr = drive - strafe + turn;

        // If any wheel is over 100%, scale them all down together so the robot
        // still moves in the right direction
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lr), Math.abs(rr)));
        if (max > 1.0) {
            lf /= max; rf /= max; lr /= max; rr /= max;
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }

    // Motor names and directions come from Pedro's Constants.java -- the ones
    // confirmed on the robot -- so there's only one place to change them.
    private void initDriveMotors() {
        MecanumConstants d = Constants.driveConstants;
        leftFront  = hardwareMap.get(DcMotor.class, d.getLeftFrontMotorName());
        leftRear   = hardwareMap.get(DcMotor.class, d.getLeftRearMotorName());
        rightFront = hardwareMap.get(DcMotor.class, d.getRightFrontMotorName());
        rightRear  = hardwareMap.get(DcMotor.class, d.getRightRearMotorName());

        leftFront.setDirection(d.getLeftFrontMotorDirection());
        leftRear.setDirection(d.getLeftRearMotorDirection());
        rightFront.setDirection(d.getRightFrontMotorDirection());
        rightRear.setDirection(d.getRightRearMotorDirection());

        // Stop hard instead of coasting when power goes to 0
        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FieldAprilTags.getLibrary())
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();
        // Look at every 2nd pixel: faster, but sees less far away. Lower to 1
        // if it can't see the tag from 36 in.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Short exposure = less blur while the robot moves (from the SDK sample)
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);
    }

    private void setManualExposure(int exposureMS, int gain) {
        telemetry.addLine("Camera starting...");
        telemetry.update();
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (isStopRequested()) return;

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        visionPortal.getCameraControl(GainControl.class).setGain(gain);
        sleep(20);
    }
}
