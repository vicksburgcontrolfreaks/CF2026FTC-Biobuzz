package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.geometry.TagLocalizer;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.geometry.VisionPoseFilter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

/**
 * Does a tag sighting actually tell us where the ROBOT is? Read-only bring-up
 * for TagLocalizer + VisionPoseFilter.
 *
 * WHY THIS EXISTS
 * TagLocalizer and VisionPoseFilter are both written and both unit-tested on a
 * laptop, and NOTHING CALLS EITHER ONE. The reason is one line: TurretCamera
 * builds its processor with FieldAprilTags.getLibrary() and no setCameraPose(),
 * so detection.robotPose is null and TagLocalizer.solve() returns null on its
 * first statement. Until a camera is configured for LOCALIZATION, none of that
 * code can run at all.
 *
 * This op mode configures one -- its own, so it changes nothing about the way
 * DriveTeleOp aims in a match -- and puts the whole chain on the screen.
 *
 * WHAT IT DOES NOT DO
 * It never writes the pose. It never moves the robot. It computes what a
 * correction WOULD be and shows whether VisionPoseFilter would accept it. The
 * blend into Pedro's pose is the next step, and it should not be written until
 * the numbers here look right.
 *
 * IT OWNS THE WEBCAM. A webcam can only be opened once, so this cannot run at
 * the same time as DriveTeleOp or anything else using TurretCamera.
 *
 * THE TURRET MUST BE HOMED. The camera is on the turret, so turning a tag
 * sighting into a robot pose needs the turret's angle
 * (TurretGeometry.robotPoseFromTurret). An unhomed turret's angle is measured
 * from wherever it powered up, which makes every fix wrong by that much. Press
 * Y to home. Nothing moves before that.
 *
 * HOW TO USE IT (this is the Phase 3 gate TagLocalizer's comment asks for)
 *   1. Park the robot on a spot you can measure with a tape. Point the camera
 *      at a hive.
 *   2. Press Y, let the turret home.
 *   3. Type the measured position into TRUTH_X / TRUTH_Y / TRUTH_HEADING_DEG
 *      in Panels, or just read the Fix line and compare by hand.
 *   4. Read "Error". That is the whole question: how far wrong is a tag fix?
 *   5. Move to another spot and do it again. Near, far, left, right, and off to
 *      one side -- the error being POSITION-DEPENDENT is what matters, because
 *      that is the shape of a camera-mount or hive-geometry error rather than
 *      noise.
 *
 * WHAT TO DISTRUST
 * HiveTags still carries made-up numbers (RED_HIVE_X, CELL_ARM, CELL_DROP), and
 * TagLocalizer's CAMERA_LEFT and CAMERA_YAW_DEG are assumed 0 on both robots. A
 * fix is only as good as those. "Believable" only checks that the turret base
 * came out level and on the floor -- it does NOT mean the fix is accurate.
 *
 * Runs on AUTO EXPOSURE, unlike TurretCamera's manual 6 ms. That is fine here
 * and only here: the robot is parked, so there is no motion blur to beat.
 */
@Configurable
@TeleOp(name = "Tag Pose Test", group = "Test")
public class TagPoseTest extends OpMode {

    /**
     * The tape-measured truth for the spot the robot is parked on, in Pedro
     * field inches. Leave TRUTH_X at NaN and the error lines are hidden -- the
     * Fix is still shown, so the op mode is useful before you have a tape out.
     */
    public static double TRUTH_X = Double.NaN;
    public static double TRUTH_Y = Double.NaN;
    public static double TRUTH_HEADING_DEG = Double.NaN;

    private RobotHardware hw;
    private Follower follower;
    private Turret turret;
    private TagLocalizer localizer;

    private AprilTagProcessor processor;
    private VisionPortal portal;

    private boolean homingStarted = false;

    // For VisionPoseFilter.check(), which wants to know if the robot is moving.
    private final ElapsedTime loopTimer = new ElapsedTime();
    private Pose lastPose = null;
    private double speed = 0, turnRate = 0;

    // The last fix we got, kept so the screen doesn't blank between frames.
    private TagLocalizer.Fix lastFix = null;
    private VisionPoseFilter.Verdict lastVerdict = null;
    private double lastFixAgeMs = 0;
    private int framesWithFix = 0, framesWithoutPose = 0;

    @Override
    public void init() {
        // Per-robot turret geometry and CAMERA numbers, before anything is solved.
        RobotProfile.install();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        hw = new RobotHardware();
        hw.init(hardwareMap, false);   // we open the webcam ourselves, below
        turret = hw.turret;
        follower = Constants.createFollower(hardwareMap);
        localizer = new TagLocalizer(TurretGeometry.forRobot());

        // THE WHOLE POINT: the localization library (tags know where they are on
        // the field) plus the camera's pose on the turret. Without both of these
        // the SDK leaves detection.robotPose null.
        processor = TagLocalizer.configure(new AprilTagProcessor.Builder())
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("TAG POSE TEST -- read only, writes nothing");
        BuildInfo.show(telemetry);
        telemetry.addData("Robot", WhichRobot.number());
        telemetry.addData("Camera", portal.getCameraState());
        telemetry.addLine();
        telemetry.addLine("Park on a spot you can measure. Point the camera at a hive.");
        telemetry.addLine("At Start, press Y to HOME THE TURRET -- fixes are wrong until you do.");
        telemetry.addLine("Nothing moves until then.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Pedro needs some starting pose to integrate from. It is a placeholder
        // and the error lines below do NOT depend on it -- the fix is compared
        // against your tape measurement, not against odometry. Odometry is shown
        // only so you can watch the two side by side.
        follower.setStartingPose(new Pose(TagLocalizer.CAMERA_FORWARD, 0, 0));
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        follower.update();

        if (gamepad1.yWasPressed() && !homingStarted && Turret.isMeasured()) {
            turret.startHoming();
            homingStarted = true;
        }
        turret.update();

        Pose pose = follower.getPose();
        if (lastPose != null && dt > 1e-6) {
            speed = Math.hypot(pose.getX() - lastPose.getX(), pose.getY() - lastPose.getY()) / dt;
            // wrapDegrees takes and returns DEGREES, so convert first, then wrap.
            turnRate = TurretGeometry.wrapDegrees(
                    Math.toDegrees(pose.getHeading() - lastPose.getHeading())) / dt;
        }
        lastPose = pose;

        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null) detections = Collections.emptyList();

        // How many tags the SDK gave a field pose for. ZERO while any tag is in
        // frame is the "camera not configured for localization" symptom, and the
        // first thing to check if this op mode shows nothing.
        int withPose = 0;
        for (AprilTagDetection d : detections) {
            if (d.robotPose != null) withPose++;
        }

        TagLocalizer.Fix fix = localizer.solveFrame(detections, turret.getAngle());
        if (fix != null) {
            lastFix = fix;
            framesWithFix++;
            lastFixAgeMs = (System.nanoTime() - fix.frameNanos) / 1e6;
            lastVerdict = VisionPoseFilter.check(turret.isOnTarget(), speed, turnRate,
                    lastFixAgeMs, fix.tagRange, pose, fix.robot);
        } else if (!detections.isEmpty() && withPose == 0) {
            framesWithoutPose++;
        }

        showTelemetry(detections.size(), withPose, dt);
    }

    private void showTelemetry(int tagsSeen, int withPose, double dt) {
        BuildInfo.show(telemetry);
        if (!turret.isHomed()) {
            telemetry.addLine(">>> TURRET NOT HOMED -- press Y. Every fix below is");
            telemetry.addLine("    wrong by however far the turret is from its true zero.");
        }
        telemetry.addData("Turret", "%s  %+.1f deg", turret.getState(), turret.getAngle());
        telemetry.addData("Tags", "%d in frame, %d with a field pose", tagsSeen, withPose);
        if (tagsSeen > 0 && withPose == 0) {
            telemetry.addLine(">>> Tags seen but NO field pose: the processor is not");
            telemetry.addLine("    configured for localization. TagLocalizer can't run.");
        }
        telemetry.addData("Frames", "%d with a fix, %d with tags but no pose",
                framesWithFix, framesWithoutPose);
        telemetry.addLine();

        if (lastFix == null) {
            telemetry.addLine("No fix yet. Point the camera at a hive cell's tags.");
            telemetry.addData("Loop", "%.0f ms", dt * 1000);
            telemetry.update();
            return;
        }

        Pose v = lastFix.robot;
        telemetry.addData("FIX  (vision)", "(%.1f, %.1f) %.1f deg", v.getX(), v.getY(),
                Math.toDegrees(v.getHeading()));
        Pose p = follower.getPose();
        telemetry.addData("Odometry", "(%.1f, %.1f) %.1f deg", p.getX(), p.getY(),
                Math.toDegrees(p.getHeading()));
        telemetry.addData("Fix quality", "%d tags, spread %.2f in, range %.1f in, %.0f ms old",
                lastFix.tagsUsed, lastFix.spread, lastFix.tagRange, lastFixAgeMs);
        telemetry.addData("Turret base", "level err %.1f deg, floor err %.2f in, hive %s%s",
                lastFix.turret.levelErrorDeg, lastFix.turret.floorError, lastFix.turret.hiveTilt,
                lastFix.turret.believable ? "" : "   <-- NOT BELIEVABLE");
        // Which rule would stop this correction being applied. Parked and settled
        // it should read ACCEPT; ROBOT_MOVING while stationary means the pose is
        // jittering, TURRET_MOVING means the turret has not settled.
        telemetry.addData("Filter", "%s   (speed %.1f in/s, turn %.1f deg/s)",
                lastVerdict, speed, turnRate);

        if (!Double.isNaN(TRUTH_X) && !Double.isNaN(TRUTH_Y)) {
            double ex = v.getX() - TRUTH_X;
            double ey = v.getY() - TRUTH_Y;
            telemetry.addLine();
            telemetry.addData("Error  <-- THE ANSWER", "%.1f in   (dx %+.1f, dy %+.1f)",
                    Math.hypot(ex, ey), ex, ey);
            if (!Double.isNaN(TRUTH_HEADING_DEG)) {
                telemetry.addData("Heading error", "%+.1f deg", TurretGeometry.wrapDegrees(
                        Math.toDegrees(v.getHeading()) - TRUTH_HEADING_DEG));
            }
        } else {
            telemetry.addLine();
            telemetry.addLine("Set TRUTH_X / TRUTH_Y in Panels to see the error directly.");
        }
        telemetry.addData("Loop", "%.0f ms", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }
}
