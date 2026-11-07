package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Shooting Sequence bring-up: everything that decides a shot, in one op mode so
 * it can be tuned against itself instead of one piece at a time.
 *
 *   pivot      the gap between the flywheels (pollen vs. nectar)
 *   flywheels  how hard the ball leaves
 *   deflector  what angle it leaves at
 *   indexer    when the ball gets fed in
 *   collector  getting a ball to the indexer in the first place
 *
 * Each of these already had its own test op mode (Pivot Jog Test, Shooter
 * Flywheel Test, Indexer Load Test). Those stay -- they do things this does not,
 * like the flywheel peak-RPM hold and the indexer current logging. What they
 * cannot do is find a combination, because a shot is all four at once.
 *
 * THE PIVOT IS A POSITIONAL SERVO. THE DEFLECTOR IS NOT.
 * They look similar and behave nothing alike:
 *
 *   pivot      Servo. setPosition() is absolute, the servo holds it itself.
 *              But Servo.getPosition() IS NOT A SENSOR -- it only echoes the
 *              last commanded value, and reads 0.0 at op mode start no matter
 *              where the pivot physically is. Pivot Jog Test learned this the
 *              hard way: it trusted getPosition(), believed 0.0, and slammed the
 *              servo across its full travel into a hard stop.
 *   deflector  CRServo. No position command exists at all; you set a speed and
 *              read the analog wire to find out where it went. See Deflector.
 *
 * So the pivot is rate limited and refuses to move until armed, and the
 * deflector goes through its subsystem.
 *
 * WHAT IS NEW HERE: THE PIVOT'S OWN FEEDBACK WIRE
 * "pivotPos" has been wired for a while and nothing has ever read it. This op
 * mode does. It does not control anything with it -- it shows the pivot's REAL
 * shaft angle beside the commanded position, which is the only way to notice
 * the mapping has drifted (a horn slipped a spline, the linkage was re-seated)
 * rather than finding out from bad shots. Capture a reference at a hard stop
 * with gamepad1 X and every later reading is a delta from it.
 *
 * Driver Station config: "shooterPivot", "shooterLeft", "shooterRight",
 * "indexer", "deflector", "deflectorPos", and optionally "pivotPos".
 *
 * DURING INIT, before anything is written to the servo
 *   dpad up / down       move the position ARM will command
 *
 * Set that near where the pivot actually is. ARM commands it at whatever speed
 * the servo likes and nothing can slow that down, so a value past a hard stop
 * slams into it -- which is what the per-robot defaults did on 5618.
 *
 * CONTROLS -- gamepad1, the shot shape
 *   Y                    ARM. Nothing moves before this. Commands the position
 *                        set during INIT; expect one jump.
 *   bumpers              pivot one fine step  (PIVOT_FINE_STEP)
 *   dpad left / right    pivot one big step   (PIVOT_BIG_STEP)
 *   B                    go wide open. The escape hatch if ARM landed on the
 *                        closed stop -- rate limited, so a traverse not a slam.
 *   X                    capture the pivot reference here (do this at a stop)
 *   left stick click     capture this as the POLLEN position
 *   right stick click    capture this as the NECTAR position
 *   A                    flywheels on / off
 *   dpad up / down       flywheel target RPM +/- RPM_STEP
 *
 * The pivot steps, it does not jog. One press is one known change with no loop
 * time in the arithmetic. The triggers used to jog it and were reported
 * intermittent; most of that was the servo being programmed for continuous
 * rotation internally while the config said positional, so setPosition() was
 * setting SPEED and 0.5 meant stop. Stepping is also just easier to tune in.
 *
 * CONTROLS -- gamepad2, the shot itself
 *   triggers             jog the deflector. Needs no arming; it has no presets
 *                        that can slam.
 *   Y                    call the deflector's position here ZERO
 *   B                    home the deflector to its zero hard stop (contingency
 *                        for auto not having run -- see Deflector)
 *   dpad up              deflector to SHOOT_ANGLE_DEG
 *   dpad down            deflector to zero
 *   A                    indexer on / off  (gated -- see below)
 *   bumpers              indexer target RPM +/- INDEXER_RPM_STEP
 *   X (held)             override the flywheel gate and feed anyway
 *   dpad left            collector on / off
 *   dpad right (held)    collector EJECT, for clearing a jam
 *
 * THE INDEXER GATE
 * Feeding a ball into flywheels that are not up to speed either jams it or
 * dribbles it out, and both look like a pivot problem from the driver station.
 * So the indexer will not run unless both flywheels are within
 * FLYWHEEL_READY_RPM of target. Hold gamepad2 X to feed regardless, which is
 * what you want when testing the indexer on its own with the shooter off.
 */
@Configurable
@TeleOp(name = "Shooting Sequence", group = "Test")
public class ShootingSequence extends LinearOpMode {

    // ---- Pivot -------------------------------------------------------------
    /**
     * What ARM commands, for both robots. 0.544 is the pollen position found by
     * stepping up from the closed hard stop in Pivot Finder, 2026-11-06 --
     * PROVISIONAL, found by eye and not yet confirmed by actually shooting,
     * which is what this op mode is for.
     *
     * Replaced the old per-robot 0.42/0.6 guesses inherited from Pivot Jog Test.
     * Those were only ever guesses at a safe place to start, and a measured
     * position beats a guess on either robot. Still adjustable with dpad up/down
     * during INIT, before anything is written to the servo.
     */
    public static double PIVOT_ARM_POSITION = 0.544;

    /**
     * Travel limit for jogging. Deliberately NOT ShooterPivot's band, because the
     * two records in this repo disagree:
     *
     *   ShooterPivot   MIN_SAFE 0.27  MAX_SAFE 0.744
     *   Pivot Jog Test header: "the 0.166-0.679 band measured 2026-10-30"
     *
     * 0.744 is past the other record's closed end, and 0.166 is past the other
     * record's open end, so trusting either one alone can drive into a stop.
     * These default to the overlap, which cannot exceed either. Widen them once
     * you have measured the band on THIS robot -- that is what the stick-click
     * captures and the pivotPos readout are for.
     */
    public static double PIVOT_MIN_POSITION = 0.27;
    public static double PIVOT_MAX_POSITION = 0.679;

    /** One bumper press, in servo position units. */
    public static double PIVOT_FINE_STEP = 0.002;

    /** One dpad left/right press. */
    public static double PIVOT_BIG_STEP = 0.01;

    /**
     * Ceiling on how fast the commanded position may change, per second. Must
     * stay above the step sizes so a press is not throttled.
     */
    public static double PIVOT_MAX_RATE = 0.20;

    // ---- Flywheels ---------------------------------------------------------
    public static double RPM_STEP = 100.0;
    public static double MAX_TARGET_RPM = 2000.0;

    /** Both wheels inside this of target counts as ready to be fed. */
    public static double FLYWHEEL_READY_RPM = 150.0;

    // ---- Indexer -----------------------------------------------------------
    public static double INDEXER_START_RPM = 50.0;
    public static double INDEXER_RPM_STEP = 10.0;
    public static double INDEXER_MAX_RPM = 300.0;

    // ---- Collector ---------------------------------------------------------
    /** Intake power when the collector is enabled. */
    public static double COLLECTOR_POWER = 0.8;

    /** Eject power, for clearing a jam. Held, not toggled. */
    public static double COLLECTOR_EJECT_POWER = 0.6;

    private static final double TICKS_PER_MOTOR_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double SHOOTER_RATIO = RobotHardware.SHOOTER_GEAR_RATIO;
    private static final double INDEXER_RATIO = RobotHardware.INDEXER_GEAR_RATIO;

    private Servo pivot;
    private AnalogInput pivotPos;          // optional
    private DcMotorEx shooterLeft, shooterRight;
    private DcMotorEx indexer;
    private DcMotorEx collector;
    private final Deflector deflector = new Deflector();

    private boolean armed = false;
    private double armPosition;
    private double pivotTarget;
    private double pivotCommanded;

    private boolean flywheelsOn = false;
    private double flywheelTargetRpm = 0;

    private boolean indexerOn = false;
    private double indexerTargetRpm = INDEXER_START_RPM;

    private boolean collectorOn = false;

    private double pivotReferenceVolts = Double.NaN;

    // Captured goldilocks positions: the commanded value AND the shaft angle the
    // wire reported there. The position is what the code will use; the voltage is
    // what proves it still means the same physical gap after a rebuild.
    private double pollenCapture = Double.NaN, pollenVolts = Double.NaN;
    private double nectarCapture = Double.NaN, nectarVolts = Double.NaN;

    private final ElapsedTime loop = new ElapsedTime();
    private boolean lastY1, lastX1, lastA1, lastRb1, lastLb1, lastDl1, lastDr1, lastDu1, lastDd1;
    private boolean lastLs1, lastRs1, lastB1;
    private boolean lastY2, lastB2, lastA2, lastRb2, lastLb2, lastDu2, lastDd2;
    private boolean lastDpl2;

    @Override
    public void runOpMode() {
        pivot = hardwareMap.get(Servo.class, "shooterPivot");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        deflector.init(hardwareMap);

        // Optional: nothing else has ever read this wire, so do not make the op
        // mode refuse to load on a robot that has not got it configured.
        pivotPos = hardwareMap.tryGet(AnalogInput.class, "pivotPos");

        // BOTH FORWARD. indexer FORWARD feeds UP toward the shooter; collector
        // FORWARD pulls pieces IN. Set here because this op mode takes the
        // motors straight from the hardwareMap, and direction lives in code,
        // never in the Driver Station config.
        //
        // THE COLLECTOR ONE MATTERS. RobotHardware on main still says REVERSE,
        // which is wrong: the 2026-10-29 check that "confirmed" REVERSE had
        // actually read the motors raw at their FORWARD default. That ran the
        // collector backwards in everything built on RobotHardware until it
        // turned up on 6494 on 2026-11-05. shane/consolidate fixes it and adds a
        // shared RobotHardware.applyCollectionDirections(); switch to that and
        // delete these two lines when it reaches main.
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        collector.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPosition = startPosition();
        pivotTarget = armPosition;
        pivotCommanded = armPosition;

        // ARM commands a position at whatever speed the servo likes, and nothing
        // in software can slow that down -- it cannot know where the pivot is, so
        // it cannot creep there. The per-robot defaults are GUESSES (Pivot Jog
        // Test says so for 5618), and a guess past a hard stop slams. So the arm
        // position is adjustable HERE, before anything is written to the servo.
        boolean lastUp = false, lastDown = false;
        while (opModeInInit()) {
            deflector.update();
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !lastUp) armPosition += 0.02;
            if (down && !lastDown) armPosition -= 0.02;
            armPosition = Range.clip(armPosition, 0.05, 0.95);
            lastUp = up; lastDown = down;

            pivotTarget = armPosition;
            pivotCommanded = armPosition;

            telemetry.addLine("NOT ARMED -- the pivot is not being written to at all.");
            telemetry.addLine();
            telemetry.addData("ARM will command", "%.3f   <-- dpad up/down to change", armPosition);
            telemetry.addData("Robot", "%s   (PIVOT_ARM_POSITION %.3f)", robotName(), PIVOT_ARM_POSITION);
            telemetry.addLine();
            telemetry.addLine("Set this NEAR WHERE THE PIVOT IS NOW, not at an end of travel.");
            telemetry.addLine("If a previous run slammed and buzzed, the stop is on that side:");
            telemetry.addLine("move this the other way before arming.");
            telemetry.addLine();
            showPivotFeedback();
            telemetry.addLine();
            telemetry.addLine("START, then gamepad1 Y to arm.  B at any time = go wide open.");
            telemetry.update();
        }

        loop.reset();
        while (opModeIsActive()) {
            double dt = loop.seconds();
            loop.reset();

            deflector.update();
            handleGamepad1(dt);
            handleGamepad2();
            driveFlywheels();
            driveIndexer();
            driveCollector();
            showTelemetry();
            telemetry.update();
        }

        indexer.setPower(0);
        collector.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        deflector.stop();
    }

    // ---- gamepad1: pivot and flywheels -------------------------------------

    private void handleGamepad1(double dt) {
        boolean y = gamepad1.y, x = gamepad1.x, a = gamepad1.a;
        boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
        boolean dl = gamepad1.dpad_left, dr = gamepad1.dpad_right;
        boolean du = gamepad1.dpad_up, dd = gamepad1.dpad_down;

        if (y && !lastY1 && !armed) {
            armed = true;
            pivotTarget = armPosition;
            pivotCommanded = armPosition;
            pivot.setPosition(pivotCommanded);
        }

        // ESCAPE HATCH. If ARM landed past the closed stop, the pivot is leaning
        // on it and walking the triggers back across the dead zone is slow and
        // feels like nothing is happening. B drives straight for wide open.
        // Still rate limited, so it is a traverse you can hear, not a slam.
        if (gamepad1.b && !lastB1 && armed) {
            pivotTarget = ShooterPivot.OPEN_POSITION;
        }

        if (x && !lastX1 && pivotPos != null) {
            pivotReferenceVolts = pivotPos.getVoltage();
        }

        // Capture the goldilocks positions. Stick buttons rather than face
        // buttons so there is no chance of hitting one while reaching for A.
        if (gamepad1.left_stick_button && !lastLs1 && armed) {
            pollenCapture = pivotCommanded;
            pollenVolts = pivotPos == null ? Double.NaN : pivotPos.getVoltage();
        }
        if (gamepad1.right_stick_button && !lastRs1 && armed) {
            nectarCapture = pivotCommanded;
            nectarVolts = pivotPos == null ? Double.NaN : pivotPos.getVoltage();
        }

        if (a && !lastA1) {
            flywheelsOn = !flywheelsOn;
        }
        if (du && !lastDu1) {
            flywheelTargetRpm = Math.min(MAX_TARGET_RPM, flywheelTargetRpm + RPM_STEP);
        }
        if (dd && !lastDd1) {
            flywheelTargetRpm = Math.max(0, flywheelTargetRpm - RPM_STEP);
        }

        if (armed) {
            // DISCRETE STEPS, NOT ANALOG JOG. One press, one known change, no
            // loop time in the arithmetic. The analog version here was reported
            // as intermittent -- most of that turned out to be the servo being
            // programmed CR (0.5 = neutral), but stepping is what was verified
            // working afterwards in Pivot Finder, so this matches it.
            if (rb && !lastRb1) pivotTarget += PIVOT_FINE_STEP;
            if (lb && !lastLb1) pivotTarget -= PIVOT_FINE_STEP;
            if (dl && !lastDl1) pivotTarget += PIVOT_BIG_STEP;
            if (dr && !lastDr1) pivotTarget -= PIVOT_BIG_STEP;

            pivotTarget = Range.clip(pivotTarget, PIVOT_MIN_POSITION, PIVOT_MAX_POSITION);

            // Still creeps toward the target, which only matters now for the B
            // escape hatch: that is the one command that can be a long traverse.
            double step = PIVOT_MAX_RATE * dt;
            pivotCommanded += Range.clip(pivotTarget - pivotCommanded, -step, step);
            pivot.setPosition(pivotCommanded);
        }

        lastY1 = y; lastX1 = x; lastA1 = a; lastRb1 = rb; lastLb1 = lb;
        lastDl1 = dl; lastDr1 = dr; lastDu1 = du; lastDd1 = dd;
        lastLs1 = gamepad1.left_stick_button; lastRs1 = gamepad1.right_stick_button;
        lastB1 = gamepad1.b;
    }

    // ---- gamepad2: deflector and indexer -----------------------------------

    private void handleGamepad2() {
        boolean y = gamepad2.y, b = gamepad2.b, a = gamepad2.a;
        boolean rb = gamepad2.right_bumper, lb = gamepad2.left_bumper;
        boolean du = gamepad2.dpad_up, dd = gamepad2.dpad_down;

        if (y && !lastY2) deflector.zeroHere();
        if (b && !lastB2) deflector.startHoming();
        if (du && !lastDu2) deflector.setTargetAngle(Deflector.SHOOT_ANGLE_DEG);
        if (dd && !lastDd2) deflector.setTargetAngle(Deflector.ZERO_ANGLE_DEG);

        // Jogging wins over a running preset: grabbing the trigger is how the
        // driver takes it back.
        double jog = gamepad2.right_trigger - gamepad2.left_trigger;
        if (Math.abs(jog) > 0.05) {
            deflector.setRawPower(jog * Deflector.MAX_POWER);
        } else if (deflector.getControlMode() == Deflector.ControlMode.JOG) {
            deflector.setRawPower(0);
        }

        if (a && !lastA2) indexerOn = !indexerOn;

        // Collector: toggle to intake, HOLD dpad right to eject. Eject is held
        // rather than toggled because it exists to clear a jam, and a stuck-on
        // eject throws away everything you just collected.
        if (gamepad2.dpad_left && !lastDpl2) collectorOn = !collectorOn;
        if (rb && !lastRb2) {
            indexerTargetRpm = Math.min(INDEXER_MAX_RPM, indexerTargetRpm + INDEXER_RPM_STEP);
        }
        if (lb && !lastLb2) {
            indexerTargetRpm = Math.max(0, indexerTargetRpm - INDEXER_RPM_STEP);
        }

        lastY2 = y; lastB2 = b; lastA2 = a; lastRb2 = rb; lastLb2 = lb;
        lastDu2 = du; lastDd2 = dd;
        lastDpl2 = gamepad2.dpad_left;
    }

    // ---- outputs ------------------------------------------------------------

    private void driveFlywheels() {
        if (flywheelsOn && flywheelTargetRpm > 0) {
            double ticksPerSec = flywheelTargetRpm * TICKS_PER_MOTOR_REV * SHOOTER_RATIO / 60.0;
            shooterLeft.setVelocity(ticksPerSec);
            shooterRight.setVelocity(ticksPerSec);
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
    }

    /**
     * Collector. Ungated: it only brings pieces in, so there is nothing for it
     * to be unsafe with. Eject wins over intake if both are asked for.
     */
    private void driveCollector() {
        if (gamepad2.dpad_right) {
            collector.setPower(-COLLECTOR_EJECT_POWER);
        } else if (collectorOn) {
            collector.setPower(COLLECTOR_POWER);
        } else {
            collector.setPower(0);
        }
    }

    private void driveIndexer() {
        if (indexerOn && (flywheelsReady() || gamepad2.x)) {
            indexer.setVelocity(indexerTargetRpm * TICKS_PER_MOTOR_REV * INDEXER_RATIO / 60.0);
        } else {
            indexer.setPower(0);
        }
    }

    /** Both wheels within FLYWHEEL_READY_RPM of a non-zero target. */
    private boolean flywheelsReady() {
        if (!flywheelsOn || flywheelTargetRpm <= 0) return false;
        return Math.abs(flywheelRpm(shooterLeft.getVelocity()) - flywheelTargetRpm) < FLYWHEEL_READY_RPM
                && Math.abs(flywheelRpm(shooterRight.getVelocity()) - flywheelTargetRpm) < FLYWHEEL_READY_RPM;
    }

    // ---- telemetry ----------------------------------------------------------

    private void showTelemetry() {
        telemetry.addData("Robot", robotName());
        telemetry.addData("ARMED", armed ? "yes" : "NO -- gamepad1 Y to arm");
        telemetry.addLine();

        telemetry.addLine("--- PIVOT ---");
        telemetry.addData("Asked servo for", "%.3f   (heading for %.3f)",
                pivotCommanded, pivotTarget);
        // ShooterPivot's POLLEN_POSITION is still the old 0.67 and has not been
        // updated to the measured 0.544, because that is what this op mode is
        // meant to confirm. Shown side by side so the difference is obvious.
        telemetry.addData("Armed at", "%.3f   (ShooterPivot still says pollen %.3f)",
                PIVOT_ARM_POSITION, ShooterPivot.POLLEN_POSITION);
        telemetry.addData("Steps", "bumpers %.3f   dpad L/R %.3f",
                PIVOT_FINE_STEP, PIVOT_BIG_STEP);
        telemetry.addData("Limited to", "%.3f .. %.3f%s",
                PIVOT_MIN_POSITION, PIVOT_MAX_POSITION,
                (pivotTarget <= PIVOT_MIN_POSITION + 1e-6 ? "   AT OPEN LIMIT"
                        : pivotTarget >= PIVOT_MAX_POSITION - 1e-6 ? "   AT CLOSED LIMIT" : ""));
        showPivotFeedback();
        if (!Double.isNaN(pollenCapture)) {
            telemetry.addData("Captured pollen", "%.3f   (%.3f V)", pollenCapture, pollenVolts);
        }
        if (!Double.isNaN(nectarCapture)) {
            telemetry.addData("Captured nectar", "%.3f   (%.3f V)", nectarCapture, nectarVolts);
        }
        if (!Double.isNaN(pollenCapture) && !Double.isNaN(nectarCapture)) {
            // Volts per position unit, from two real points. This is the number
            // that converts "move N degrees" into a setPosition() value, and the
            // only honest way to get it is to measure it like this.
            double dPos = nectarCapture - pollenCapture;
            telemetry.addData("Pollen -> nectar", "%+.3f pos   %+.3f V%s",
                    dPos, nectarVolts - pollenVolts,
                    Math.abs(dPos) > 1e-6
                            ? String.format("   =  %.3f V per pos unit",
                                    (nectarVolts - pollenVolts) / dPos)
                            : "");
        }

        telemetry.addLine();
        telemetry.addLine("--- FLYWHEELS ---");
        telemetry.addData("State", flywheelsOn ? "ON" : "off");
        telemetry.addData("Target", "%.0f RPM", flywheelTargetRpm);
        telemetry.addData("Actual", "L %.0f   R %.0f   (gap %.0f)",
                flywheelRpm(shooterLeft.getVelocity()),
                flywheelRpm(shooterRight.getVelocity()),
                flywheelRpm(shooterLeft.getVelocity()) - flywheelRpm(shooterRight.getVelocity()));
        telemetry.addData("Up to speed", flywheelsReady() ? "YES" : "no");

        telemetry.addLine();
        telemetry.addLine("--- INDEXER ---");
        telemetry.addData("State", indexerOn
                ? (flywheelsReady() ? "FEEDING"
                        : (gamepad2.x ? "FEEDING (gate overridden)" : "on, HELD -- flywheels not up to speed"))
                : "off");
        telemetry.addData("Target", "%.0f RPM", indexerTargetRpm);

        telemetry.addLine();
        telemetry.addLine("--- COLLECTOR ---");
        telemetry.addData("State", gamepad2.dpad_right ? "EJECTING"
                : collectorOn ? "ON (pulling in)" : "off");
        telemetry.addData("Direction", "FORWARD = in. RobotHardware on main still "
                + "says REVERSE, which is the known-wrong value");

        telemetry.addLine();
        telemetry.addLine("--- DEFLECTOR ---");
        telemetry.addData("State", "%s / %s", deflector.getState(), deflector.getControlMode());
        if (deflector.isReady()) {
            telemetry.addData("Angle", "%.0f deg   (target %.0f)",
                    deflector.getAngle(), deflector.getTargetAngle());
        } else {
            telemetry.addData("Angle", "unknown -- Y to zero here, or B to home");
        }
        telemetry.addData("Within one turn", "%.1f deg   %.3f V",
                deflector.getRawAngle(), deflector.getVoltage());
        telemetry.addData("Direction", deflector.isSignLearned()
                ? String.format("%+.0f, confirmed by watching a move", deflector.getEffectiveSign())
                : String.format("%+.0f, still a guess until the first move",
                        deflector.getEffectiveSign()));
        if (deflector.getFailure() != null) {
            telemetry.addLine(deflector.getFailure());
        }
    }

    /** The pivot's real shaft angle, if the wire is configured. */
    private void showPivotFeedback() {
        if (pivotPos == null) {
            telemetry.addData("pivotPos", "not configured -- commanded position is all there is");
            return;
        }
        double volts = pivotPos.getVoltage();
        double max = pivotPos.getMaxVoltage();
        telemetry.addData("pivotPos", "%.3f V  (of %.3f V)", volts, max);
        if (Double.isNaN(pivotReferenceVolts)) {
            telemetry.addData("Reference", "none -- park at a hard stop and press gamepad1 X");
        } else {
            double deltaVolts = volts - pivotReferenceVolts;
            telemetry.addData("From reference", "%+.3f V  =  %+.1f deg",
                    deltaVolts, max > 0 ? deltaVolts / max * 360.0 : 0);
        }
    }

    private double flywheelRpm(double motorTicksPerSec) {
        return (motorTicksPerSec / (TICKS_PER_MOTOR_REV * SHOOTER_RATIO)) * 60.0;
    }

    private static double startPosition() {
        return PIVOT_ARM_POSITION;
    }

    /**
     * For display only. WhichRobot throws when the config name does not start
     * with a robot number, which is right for drive code -- driving the wrong
     * way is worse than refusing to start. But nothing here depends on the
     * answer any more, so a label must not be able to kill the op mode partway
     * through a shooting test.
     */
    private static String robotName() {
        try {
            return WhichRobot.number();
        } catch (RuntimeException e) {
            return "unknown (config name does not start with 5618 or 6494)";
        }
    }
}
