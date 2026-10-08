package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * SAFETY: it only moves while the left bumper is held AND it can see the tag.
 * Lose the tag -> it stops. It never goes looking for it.
 *
 * Based on the FTC SDK sample RobotAutoDriveToAprilTagOmni. Each loop it
 * measures three "errors" and fixes each one with a different wheel motion:
 *   - too far / too close           -> drive forward / back
 *   - tag off to one side of camera -> turn to point at it
 *   - not square to the tag's face  -> strafe sideways around it
 * Each fix is error x GAIN, so a big error = a big push, small error = gentle.
 */
@TeleOp(name = "AprilTag Follow", group = "Test")
public class AprilTagFollow extends LinearOpMode {

    // ---------------- WHAT TO FOLLOW ----------------
    private static final int    TARGET_TAG_ID   = 30;   // any other tag is ignored
    private static final double TARGET_DISTANCE = 36.0; // inches, camera to tag, along the floor

    // ---------------- WHERE THE CAMERA IS ----------------
    // Measured on the robot. Centered left/right, facing straight forward.
    private static final double CAMERA_HEIGHT    = 18.0; // inches above the floor
    // How far the camera is tipped UP (0 = level). On the field the tags are on
    // the BOTTOM of the hive cells, so a hive camera will probably tip up.
    // Change this when it does -- the distance math below already uses it.
    private static final double CAMERA_PITCH_DEG = 0.0;

    // ---------------- HOW HARD TO PUSH (gains) ----------------
    // Same starting numbers as the SDK sample. If the robot wobbles back and
    // forth around the target, lower the gain. If it creeps slowly, raise it.
    private static final double DRIVE_GAIN  = 0.02;  // power per inch of distance error
    private static final double STRAFE_GAIN = 0.015; // power per degree of "not square"
    private static final double TURN_GAIN   = 0.01;  // power per degree of "not pointed at it"

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
        telemetry.addLine("HOLD left bumper to follow. dpad up/down = speed.");
        telemetry.addLine("Camera view: 3 dots menu > Camera Stream");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Speed limit: one step per press
            if (gamepad1.dpadUpWasPressed())   maxSpeed += SPEED_STEP;
            if (gamepad1.dpadDownWasPressed()) maxSpeed -= SPEED_STEP;
            maxSpeed = Range.clip(maxSpeed, MIN_SPEED, MAX_SPEED);

            AprilTagDetection tag = findTargetTag();

            double drive = 0, strafe = 0, turn = 0;

            if (tag == null) {
                telemetry.addLine("Tag " + TARGET_TAG_ID + ": NOT SEEN -> stopped");
            } else {
                TagPosition pos = whereIsTheTag(tag);

                double distanceError = pos.forward - TARGET_DISTANCE; // + = too far away
                double bearingError  = pos.bearing;                   // + = tag is to our left
                double yawError      = tag.ftcPose.yaw;               // + = we're off to one side of its face

                if (gamepad1.left_bumper) {
                    double turnLimit = maxSpeed * TURN_SHARE;
                    drive  = Range.clip(distanceError * DRIVE_GAIN,  -maxSpeed,  maxSpeed);
                    strafe = Range.clip(-yawError     * STRAFE_GAIN, -maxSpeed,  maxSpeed);
                    turn   = Range.clip(bearingError  * TURN_GAIN,   -turnLimit, turnLimit);
                }

                telemetry.addLine("Tag " + TARGET_TAG_ID + ": SEEN"
                        + (gamepad1.left_bumper ? " -> FOLLOWING" : " (hold left bumper)"));
                telemetry.addData("Distance (in)", "%5.1f  (want %.0f, error %+5.1f)",
                        pos.forward, TARGET_DISTANCE, distanceError);
                telemetry.addData("Sideways (in)", "%+5.1f  (+ = tag is to the right)", pos.right);
                telemetry.addData("Tag height (in)", "%5.1f  (above the floor)", pos.height);
                telemetry.addData("Bearing (deg)", "%+5.1f  (want 0, + = tag to the left)", bearingError);
                telemetry.addData("Yaw (deg)", "%+5.1f  (want 0, 0 = square to its face)", yawError);
                telemetry.addData("Straight-line range (in)", "%5.1f", tag.ftcPose.range);
            }

            moveRobot(drive, strafe, turn);

            telemetry.addData("Top speed", "%.2f  (dpad up/down)", maxSpeed);
            telemetry.addData("Wheels", "drive %+.2f  strafe %+.2f  turn %+.2f", drive, strafe, turn);
            telemetry.update();
        }

        moveRobot(0, 0, 0);
        visionPortal.close();
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
        setManualExposure(6, 250);
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
