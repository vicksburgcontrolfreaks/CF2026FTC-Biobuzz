package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auton.Waypoints;
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
 * It never writes the pose and it never drives the robot. It computes what a
 * correction WOULD be and shows whether VisionPoseFilter would accept it. The
 * blend into Pedro's pose is the next step, and it should not be written until
 * the numbers here look right. It DOES move the turret, but only the homing you
 * ask for with Y and the jogging you ask for yourself.
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
 *   3. Dial the measured position into the TRUTH lines at the bottom of the
 *      screen: D-pad Up/Down picks X, Y or heading, the TRIGGERS move it.
 *      No Panels needed. Parked on the garden-edge start? Press X instead.
 *   4. Read "Error". That is the whole question: how far wrong is a tag fix?
 *   5. Move to another spot and do it again. Near, far, left, right, and off to
 *      one side -- the error being POSITION-DEPENDENT is what matters, because
 *      that is the shape of a camera-mount or hive-geometry error rather than
 *      noise.
 *
 * THE TURRET SWEEP: SHOULD THE CAMERA COME OFF THE TURRET?
 * Taking the camera off the turret would allow a 360 deg turret and would delete
 * the whole tag -> turret base -> robot chain from localization. Whether that is
 * worth a rebuild depends on how much that chain is actually costing, and this
 * measures it instead of guessing:
 *
 *   1. Park the robot. Do not move it again until step 4.
 *   2. Press Y, let the turret home. Point it at a cell and press A.
 *   3. Step the turret (D-pad L/R) to another angle, let it SETTLE, press A.
 *      Repeat for 3-5 angles across the travel.
 *   4. Read SPREAD. The robot never moved, so every inch of spread is the turret
 *      chain: homing accuracy, TURRET_ZERO_DEG, TURRET_CCW_SIGN,
 *      TICKS_PER_DEGREE, TURRET_OFFSET_X/Y and gear backlash.
 *
 * A small spread means the turret chain is fine and a body-mounted camera would
 * buy little localization accuracy -- decide on the 360 deg turret alone. Inches
 * of spread is exactly what moving the camera removes.
 *
 * It also separates two failure modes: error that CHANGES with turret angle is
 * the turret chain, error that stays put is the camera mount or hive geometry.
 *
 * CONTROLS (gamepad 1)
 *   Y              home the turret (once; nothing moves before it)
 *   D-pad L / R    step the turret target by TURRET_STEP_DEG
 *   Right stick X  jog the turret (right = clockwise, as in DriveTeleOp)
 *   A              capture a sweep sample (refused unless settled and fresh)
 *   B              clear the samples
 *   D-pad U / D    pick which TRUTH value to edit (X / Y / heading)
 *   Triggers       move the picked TRUTH value (RT up, LT down)
 *   X              load GARDEN_EDGE_START as the truth (red coordinates)
 *
 * WHAT TO DISTRUST (updated 2026-11-13)
 * The HIVE is measured -- CELL_ARM, CELL_DROP, HIVE_SWING_DEG and PIVOT_Y all
 * have tape behind them now. What is still assumed is on the ROBOT:
 * TagLocalizer's CAMERA_LEFT and CAMERA_YAW_DEG are both 0 by assumption, and
 * the camera calibration is whatever the SDK picks. "Believable" only checks
 * that the turret base came out level and on the floor -- it does NOT mean the
 * fix is accurate.
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

    /** One D-pad L/R press, degrees of turret target. */
    public static double TURRET_STEP_DEG = 15.0;
    /** Right stick at full deflection, degrees of turret target per second. */
    public static double TURRET_JOG_DEG_PER_SEC = 45.0;
    /** A captured sample's fix must be newer than this, or the capture is refused. */
    public static double CAPTURE_MAX_FIX_AGE_SEC = 0.5;
    /** Inches (or degrees) per second while a trigger is held, editing the truth. */
    public static double TRUTH_EDIT_PER_SEC = 15.0;

    private static final int MAX_SAMPLES = 8;
    private static final double STICK_DEADZONE = 0.1;

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
    private final ElapsedTime sinceFix = new ElapsedTime();

    // ---- The turret sweep ---------------------------------------------------
    // Park the robot, capture a fix at several TURRET angles without moving the
    // robot, and the reported ROBOT position should not change -- the robot
    // hasn't moved. However far it does move is what the turret chain costs:
    // homing accuracy, TURRET_ZERO_DEG, TURRET_CCW_SIGN, TICKS_PER_DEGREE,
    // TURRET_OFFSET_X/Y and gear backlash, all of which a body-mounted camera
    // would delete. That is the number the fixed-vs-turret camera decision turns
    // on, so it is measured here rather than argued about.
    private final double[] sampleTurretDeg = new double[MAX_SAMPLES];
    private final double[] sampleX = new double[MAX_SAMPLES];
    private final double[] sampleY = new double[MAX_SAMPLES];
    private final double[] sampleHeadingDeg = new double[MAX_SAMPLES];
    private int samples = 0;
    private String captureNote = "A captures a sample once the turret has settled";

    // Has the odometry pose been given a real origin yet? See start(). Until it
    // has, VisionPoseFilter's "too far from odometry" rule is meaningless, so the
    // Filter line says so rather than printing a verdict nobody should act on.
    private boolean poseSeeded = false;
    private String poseOrigin = "";

    /**
     * Editing the truth FROM THE GAMEPAD, because TRUTH_X/Y are @Configurable and
     * Panels is not something everyone on this team uses. D-pad Up/Down picks the
     * field, the triggers move it, X loads the garden-edge preset.
     *
     * Tape-measuring to a robot's centre of rotation is good to about an inch at
     * best, so 1 in of editing resolution is not the limiting error here.
     */
    private enum TruthField { X, Y, HEADING }

    private TruthField editing = TruthField.X;

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
                // PINNED, and it matters. Our camera is a Logitech C270, and the
                // SDK's built-in calibration for it exists at 640x480 ONLY
                // (focalLength 822.317, distortion -0.045 / 1.173 / -3.632 --
                // see TeamCode/src/main/res/xml/teamwebcamcalibrations.xml, where
                // the same entry is commented out because the SDK ships it).
                //
                // Nothing here used to ask for a resolution, so we were trusting
                // the default to land on the calibrated one. Two measurements on
                // 2026-11-13 say it did not: range read about 10% short, and the
                // fix got ~4x worse toward the edges of the frame. Both are what
                // you get when the focal length and distortion coefficients are
                // NOT being applied, and that big a radial term should have been
                // corrected away if they were.
                //
                // If pinning this fixes those two, the same line belongs in
                // TurretCamera -- but that one feeds match aiming through
                // TurretAim, so measure here first.
                .setCameraResolution(new Size(640, 480))
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
        // WHERE ODOMETRY STARTS, AND WHY IT MATTERS HERE (fixed 2026-11-12).
        //
        // This used to seed a placeholder (CAMERA_FORWARD, 0, 0) on the grounds
        // that nothing read it. That was wrong: VisionPoseFilter's fifth rule
        // rejects a fix more than MAX_JUMP (18 in) from the odometry pose, so a
        // fictional pose made the Filter line read TOO_FAR_FROM_ODOMETRY forever
        // -- a scary-looking verdict that meant nothing. Shane hit it the first
        // time he ran this.
        //
        // So: start odometry at the TRUTH when there is one. Then the fix should
        // land within an inch or two of it, and TOO_FAR_FROM_ODOMETRY becomes a
        // real finding (the fix is more than 18 in off a spot you measured).
        // With no truth set, the FIRST fix seeds it instead -- see loop() -- and
        // the jump rule then checks later fixes against the first one, which is
        // a consistency check rather than an accuracy one.
        if (!Double.isNaN(TRUTH_X) && !Double.isNaN(TRUTH_Y)) {
            double heading = Double.isNaN(TRUTH_HEADING_DEG) ? 0 : Math.toRadians(TRUTH_HEADING_DEG);
            follower.setStartingPose(new Pose(TRUTH_X, TRUTH_Y, heading));
            poseSeeded = true;
            poseOrigin = "your TRUTH_X/Y"
                    + (Double.isNaN(TRUTH_HEADING_DEG) ? " (heading assumed 0)" : "");
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
            poseOrigin = "nothing yet -- waiting for the first fix to seed it";
        }
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
        // Jog only once homed, so the soft limits are on and a step can't drive
        // the turret into the cable or a stop. Same sign convention as
        // DriveTeleOp: stick RIGHT turns the turret right, angle goes DOWN.
        if (turret.isHomed()) {
            if (gamepad1.dpadLeftWasPressed()) {
                turret.setTargetAngle(turret.getTargetAngle() + TURRET_STEP_DEG);
            }
            if (gamepad1.dpadRightWasPressed()) {
                turret.setTargetAngle(turret.getTargetAngle() - TURRET_STEP_DEG);
            }
            double stick = gamepad1.right_stick_x;
            if (Math.abs(stick) > STICK_DEADZONE) {
                turret.setTargetAngle(turret.getTargetAngle()
                        - stick * TURRET_JOG_DEG_PER_SEC * dt);
            }
        }
        if (gamepad1.aWasPressed()) captureSample();
        if (gamepad1.bWasPressed()) {
            samples = 0;
            captureNote = "cleared";
        }
        editTruth(dt);
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
            // No truth typed in, so let the first fix be the origin. Everything
            // after it is then compared against something real, which is what
            // makes the Filter verdict worth reading at all.
            if (!poseSeeded) {
                follower.setPose(fix.robot);
                pose = fix.robot;
                lastPose = pose;
                poseSeeded = true;
                poseOrigin = "the first fix (no TRUTH set)";
            }
            lastFix = fix;
            sinceFix.reset();
            framesWithFix++;
            lastFixAgeMs = (System.nanoTime() - fix.frameNanos) / 1e6;
            lastVerdict = VisionPoseFilter.check(turret.isOnTarget(), speed, turnRate,
                    lastFixAgeMs, fix.tagRange, fix.tagBearingDeg, pose, fix.robot);
        } else if (!detections.isEmpty() && withPose == 0) {
            framesWithoutPose++;
        }

        showTelemetry(detections.size(), withPose, dt);
    }

    /**
     * Record this turret angle and the fix that goes with it.
     *
     * Refuses rather than records a meaningless sample. A fix taken mid-slew is
     * computed against a turret angle the turret has already left, and a stale
     * fix describes a turret position from before the last step -- either one
     * would show up as turret-chain error that isn't real, which is the exact
     * thing this test is trying to measure.
     */
    private void captureSample() {
        if (!turret.isHomed()) {
            captureNote = "REFUSED: turret not homed (press Y)";
        } else if (!turret.isOnTarget()) {
            captureNote = "REFUSED: turret still moving -- let it settle";
        } else if (lastFix == null || sinceFix.seconds() > CAPTURE_MAX_FIX_AGE_SEC) {
            captureNote = "REFUSED: no fresh fix -- point the camera at a cell's tags";
        } else if (samples >= MAX_SAMPLES) {
            captureNote = "REFUSED: " + MAX_SAMPLES + " samples is full (B clears)";
        } else {
            sampleTurretDeg[samples] = turret.getAngle();
            sampleX[samples] = lastFix.robot.getX();
            sampleY[samples] = lastFix.robot.getY();
            sampleHeadingDeg[samples] = Math.toDegrees(lastFix.robot.getHeading());
            samples++;
            captureNote = "captured " + samples + " of " + MAX_SAMPLES;
        }
    }

    /** Farthest captured fix from the mean of them, inches. 0 with fewer than 2. */
    private double positionSpread() {
        if (samples < 2) return 0;
        double mx = 0, my = 0;
        for (int i = 0; i < samples; i++) {
            mx += sampleX[i];
            my += sampleY[i];
        }
        mx /= samples;
        my /= samples;
        double worst = 0;
        for (int i = 0; i < samples; i++) {
            worst = Math.max(worst, Math.hypot(sampleX[i] - mx, sampleY[i] - my));
        }
        return worst;
    }

    /**
     * Dial the tape measurement in from the gamepad. See TruthField.
     *
     * X loads GARDEN_EDGE_START, which is the one spot on the field whose
     * coordinates are already known AND physically repeatable -- back edge on the
     * end of the garden tape, side touching the garden wall. Parking there needs
     * no tape at all, and it is the same spot DriveTeleOp now assumes at Start.
     * RED coordinates: this op mode has no alliance row, so on the blue side use
     * the triggers or spin the numbers through 144 yourself.
     */
    private void editTruth(double dt) {
        if (gamepad1.dpadUpWasPressed()) {
            editing = TruthField.values()[(editing.ordinal() + TruthField.values().length - 1)
                    % TruthField.values().length];
        }
        if (gamepad1.dpadDownWasPressed()) {
            editing = TruthField.values()[(editing.ordinal() + 1) % TruthField.values().length];
        }
        if (gamepad1.xWasPressed()) {
            Pose g = Waypoints.GARDEN_EDGE_START;
            TRUTH_X = g.getX();
            TRUTH_Y = g.getY();
            TRUTH_HEADING_DEG = Math.toDegrees(g.getHeading());
        }
        double adjust = (gamepad1.right_trigger - gamepad1.left_trigger) * TRUTH_EDIT_PER_SEC * dt;
        if (adjust == 0) return;
        switch (editing) {
            case X:
                TRUTH_X = (Double.isNaN(TRUTH_X) ? 0 : TRUTH_X) + adjust;
                break;
            case Y:
                TRUTH_Y = (Double.isNaN(TRUTH_Y) ? 0 : TRUTH_Y) + adjust;
                break;
            case HEADING:
                TRUTH_HEADING_DEG = (Double.isNaN(TRUTH_HEADING_DEG) ? 0 : TRUTH_HEADING_DEG) + adjust;
                break;
        }
    }

    /**
     * Two lines, at the TOP of the screen. Nothing here is saved anywhere and
     * nothing needs to be: the numbers are live, and the ERROR line below updates
     * as you move them. X OVERWRITES all three with the garden-edge start, so do
     * not press it after dialling a tape measurement in.
     */
    private void showTruthEditor() {
        telemetry.addLine(String.format("TRUTH  %sx %s   %sy %s   %shdg %s",
                editing == TruthField.X ? ">" : " ", fmt(TRUTH_X),
                editing == TruthField.Y ? ">" : " ", fmt(TRUTH_Y),
                editing == TruthField.HEADING ? ">" : " ", fmt(TRUTH_HEADING_DEG)));
        telemetry.addLine("  D-pad U/D pick | TRIGGERS move | X = garden edge (overwrites)");
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "--" : String.format("%.1f", v);
    }

    /** Turret angle covered by the captures, degrees. Near 0 = a noise-floor run. */
    private double turretSpan() {
        if (samples < 2) return 0;
        double lo = sampleTurretDeg[0], hi = sampleTurretDeg[0];
        for (int i = 1; i < samples; i++) {
            lo = Math.min(lo, sampleTurretDeg[i]);
            hi = Math.max(hi, sampleTurretDeg[i]);
        }
        return hi - lo;
    }

    /**
     * Least-squares line through (turret angle, captured heading).
     *
     * @return {slope deg/deg, intercept deg, RMS residual deg}. Slope is r - 1,
     *         where r = TICKS_PER_DEGREE assumed / true -- see showSweep().
     *         Returns zeros if the captures don't span enough turret angle to
     *         fit anything, which is the noise-floor case.
     */
    private double[] headingFit() {
        int n = samples;
        if (n < 3) return new double[] {0, 0, 0};
        double sx = 0, sy = 0, sxy = 0, sxx = 0;
        for (int i = 0; i < n; i++) {
            sx += sampleTurretDeg[i];
            sy += sampleHeadingDeg[i];
            sxy += sampleTurretDeg[i] * sampleHeadingDeg[i];
            sxx += sampleTurretDeg[i] * sampleTurretDeg[i];
        }
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-9) return new double[] {0, 0, 0};
        double slope = (n * sxy - sx * sy) / den;
        double intercept = (sy - slope * sx) / n;
        double sumSq = 0;
        for (int i = 0; i < n; i++) {
            double e = sampleHeadingDeg[i] - (slope * sampleTurretDeg[i] + intercept);
            sumSq += e * e;
        }
        return new double[] {slope, intercept, Math.sqrt(sumSq / n)};
    }

    /** Biggest gap between any two captured headings, degrees. 0 with fewer than 2. */
    private double headingSpread() {
        double worst = 0;
        for (int i = 0; i < samples; i++) {
            for (int j = i + 1; j < samples; j++) {
                worst = Math.max(worst, Math.abs(TurretGeometry.wrapDegrees(
                        sampleHeadingDeg[i] - sampleHeadingDeg[j])));
            }
        }
        return worst;
    }

    private void showSweep() {
        // One line when idle, a handful when sweeping. The per-sample list is
        // gone on purpose -- it crowded the editor and the ERROR line off the
        // Driver Station, and SPREAD is the number the test is actually for.
        if (samples == 0) {
            telemetry.addData("SWEEP", "A captures (%s)", captureNote);
            return;
        }
        telemetry.addData("SWEEP", "%d captured   (%s)", samples, captureNote);
        if (samples < 2) return;
        telemetry.addData("SPREAD", "%.2f in, heading %.1f deg",
                positionSpread(), headingSpread());

        // Which test did you just run? Captures all at ONE turret angle measure
        // the NOISE FLOOR; captures across angles measure the turret chain on top
        // of it. Telling them apart by the angle span means one op mode does both
        // and neither gets mistaken for the other -- which nearly happened on
        // 2026-11-12, when a 4.2 in across-angle spread looked like it might be
        // tag noise until 8 captures at one angle came back at 0.6 in.
        double span = turretSpan();
        if (span < 5.0) {
            telemetry.addLine(String.format(
                    "    ^ all within %.1f deg of turret: NOISE FLOOR, not the chain.", span));
            return;
        }

        // The slope of heading error against turret angle is the invariant.
        // SPREAD depends on which angles you happened to sample; the slope does
        // not, so it is what to compare between runs.
        //
        // The tag gives the turret base's field heading WITHOUT the encoder, and
        // the robot heading is that minus the reported turret angle. The robot
        // never moved, so its true heading is constant and
        //     computed heading = true heading + turretAngle * (r - 1)
        // where r = TICKS_PER_DEGREE_assumed / TICKS_PER_DEGREE_true. So the fitted
        // slope IS r - 1, and the intercept is the real heading.
        double[] fit = headingFit();
        double slope = fit[0], residual = fit[2];
        telemetry.addData("SLOPE", "%+.4f deg/deg over %.0f deg | residual %.2f deg%s",
                slope, span, residual,
                Math.abs(slope) > 0.01
                        ? String.format(" | TPD %.1f", Turret.TICKS_PER_DEGREE / (1 + slope))
                        : "");
        // How to read SLOPE: residual near the noise floor means a clean scale
        // error and the TPD figure is trustworthy; a big residual means it is not
        // a simple scale error (backlash or a nonlinearity), and capturing all
        // one direction versus mixed separates those. Kept as a comment rather
        // than four telemetry lines -- see showTelemetry() on why.
    }

    /**
     * SHORT ON PURPOSE. The Driver Station telemetry pane does not scroll -- it
     * shows what fits and silently drops the rest. The first version of this
     * screen put the TRUTH editor and the Error line at the BOTTOM of about 20
     * lines, so Shane ran the whole test on 2026-11-13 without ever seeing
     * either one, dialled in nothing, and got no error read-out.
     *
     * So: the thing you STEER and the thing you READ go first, blocking warnings
     * above them, and the diagnostics get one line each. If you add a line here,
     * take one out.
     */
    private void showTelemetry(int tagsSeen, int withPose, double dt) {
        // Blocking problems first -- nothing below means anything if these fire.
        if (!turret.isHomed()) {
            telemetry.addLine(">>> TURRET NOT HOMED -- press Y. Every fix is wrong until you do.");
        }
        if (tagsSeen > 0 && withPose == 0) {
            telemetry.addLine(">>> Tags seen but NO field pose: processor not configured");
            telemetry.addLine("    for localization. TagLocalizer cannot run. Tell Claude.");
        }

        showTruthEditor();

        Pose v = lastFix == null ? null : lastFix.robot;
        if (v != null && !Double.isNaN(TRUTH_X) && !Double.isNaN(TRUTH_Y)) {
            double ex = v.getX() - TRUTH_X;
            double ey = v.getY() - TRUTH_Y;
            telemetry.addData("ERROR  <-- write this down", "%.2f in   dx %+.2f  dy %+.2f%s",
                    Math.hypot(ex, ey), ex, ey,
                    Double.isNaN(TRUTH_HEADING_DEG) ? ""
                            : String.format("   hdg %+.1f deg", TurretGeometry.wrapDegrees(
                                    Math.toDegrees(v.getHeading()) - TRUTH_HEADING_DEG)));
        } else if (v != null) {
            telemetry.addLine("ERROR: set TRUTH above (triggers) and it appears here.");
        }

        if (v == null) {
            telemetry.addLine("NO FIX yet -- point the camera at a hive cell's tags.");
            // The counters only earn screen space here: "tags seen, never a pose"
            // is the localization-not-configured symptom, and "fixes seen, none
            // now" means it worked a moment ago and the tags have just gone.
            telemetry.addData("Tags", "%d in frame, %d with a pose   (%d fixes, %d poseless)",
                    tagsSeen, withPose, framesWithFix, framesWithoutPose);
            showSweep();
            telemetry.addData("Loop", "%.0f ms", dt * 1000);
            telemetry.update();
            return;
        }

        telemetry.addData("FIX", "(%.1f, %.1f) %.1f deg", v.getX(), v.getY(),
                Math.toDegrees(v.getHeading()));
        // "off centre" matters most: past MAX_TAG_BEARING_DEG the filter throws the
        // fix away, because an edge-of-frame tag measured 4x worse (2026-11-13).
        telemetry.addData("QUALITY", "%d tags, spread %.2f in, %.0f in away, %.0f deg off centre",
                lastFix.tagsUsed, lastFix.spread, lastFix.tagRange, lastFix.tagBearingDeg);
        telemetry.addData("FILTER", "%s%s", lastVerdict,
                lastFix.turret.believable ? "" : "   TURRET BASE NOT BELIEVABLE");
        // BASE is a free consistency check and the reason it is back on screen:
        // the turret base really is on the floor and level, so z and level should
        // solve to ~0. Several inches of z means the VERTICAL chain is wrong
        // (CAMERA_UP, CAMERA_PITCH_DEG, or the tag height), and a vertical error
        // comes with a horizontal range error -- which is the open question as of
        // 2026-11-13. I had compacted these two off the screen; that was a
        // mistake, they are the cheapest diagnostic here.
        telemetry.addData("BASE", "z %+.2f in | level %.1f deg | %s | turret %+.1f",
                lastFix.turret.z, lastFix.turret.levelErrorDeg, lastFix.turret.hiveTilt,
                turret.getAngle());
        showSweep();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }
}
