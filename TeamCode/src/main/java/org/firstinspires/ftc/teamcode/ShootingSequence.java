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
 * Shooting Sequence bring-up: the four things that decide a shot, in one op mode
 * so they can be tuned against each other instead of one at a time.
 *
 *   pivot      the gap between the flywheels (pollen vs. nectar)
 *   flywheels  how hard the ball leaves
 *   deflector  what angle it leaves at
 *   indexer    when the ball gets fed in
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
 * CONTROLS -- gamepad1, the shot shape
 *   Y                    ARM. Nothing moves before this. Commands the pivot to
 *                        START_POSITION for this robot; expect one jump.
 *   triggers             jog the pivot. RT tightens, LT widens. Rate limited.
 *   bumpers              pivot fine step, one press at a time
 *   dpad left / right    pivot to the POLLEN / NECTAR position
 *   X                    capture the pivot reference here (do this at a stop)
 *   left stick click     capture this as the POLLEN position
 *   right stick click    capture this as the NECTAR position
 *   A                    flywheels on / off
 *   dpad up / down       flywheel target RPM +/- RPM_STEP
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
    // Per robot, and deliberately mid-band rather than at an end of travel: an
    // end is the first number a reassembly invalidates, and ARM commands this
    // position at whatever speed the servo likes. Same values as Pivot Jog Test.
    public static double PIVOT_START_6494 = 0.42;
    public static double PIVOT_START_5618 = 0.6;

    /** One bumper press, in servo position units. */
    public static double PIVOT_FINE_STEP = 0.002;

    /** Full trigger, position units per second. */
    public static double PIVOT_COARSE_RATE = 0.06;

    /** Ceiling on how fast the commanded position may change, per second. */
    public static double PIVOT_MAX_RATE = 0.10;

    // ---- Flywheels ---------------------------------------------------------
    public static double RPM_STEP = 100.0;
    public static double MAX_TARGET_RPM = 2000.0;

    /** Both wheels inside this of target counts as ready to be fed. */
    public static double FLYWHEEL_READY_RPM = 150.0;

    // ---- Indexer -----------------------------------------------------------
    public static double INDEXER_START_RPM = 50.0;
    public static double INDEXER_RPM_STEP = 10.0;
    public static double INDEXER_MAX_RPM = 300.0;

    private static final double TICKS_PER_MOTOR_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double SHOOTER_RATIO = RobotHardware.SHOOTER_GEAR_RATIO;
    private static final double INDEXER_RATIO = RobotHardware.INDEXER_GEAR_RATIO;

    private Servo pivot;
    private AnalogInput pivotPos;          // optional
    private DcMotorEx shooterLeft, shooterRight;
    private DcMotorEx indexer;
    private final Deflector deflector = new Deflector();

    private boolean armed = false;
    private double pivotTarget;
    private double pivotCommanded;

    private boolean flywheelsOn = false;
    private double flywheelTargetRpm = 0;

    private boolean indexerOn = false;
    private double indexerTargetRpm = INDEXER_START_RPM;

    private double pivotReferenceVolts = Double.NaN;

    // Captured goldilocks positions: the commanded value AND the shaft angle the
    // wire reported there. The position is what the code will use; the voltage is
    // what proves it still means the same physical gap after a rebuild.
    private double pollenCapture = Double.NaN, pollenVolts = Double.NaN;
    private double nectarCapture = Double.NaN, nectarVolts = Double.NaN;

    private final ElapsedTime loop = new ElapsedTime();
    private boolean lastY1, lastX1, lastA1, lastRb1, lastLb1, lastDl1, lastDr1, lastDu1, lastDd1;
    private boolean lastLs1, lastRs1;
    private boolean lastY2, lastB2, lastA2, lastRb2, lastLb2, lastDu2, lastDd2;

    @Override
    public void runOpMode() {
        pivot = hardwareMap.get(Servo.class, "shooterPivot");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        deflector.init(hardwareMap);

        // Optional: nothing else has ever read this wire, so do not make the op
        // mode refuse to load on a robot that has not got it configured.
        pivotPos = hardwareMap.tryGet(AnalogInput.class, "pivotPos");

        // FORWARD feeds UP toward the shooter (confirmed on the robot
        // 2026-10-29). Set here because this op mode takes the motor straight
        // from the hardwareMap rather than through RobotHardware, and direction
        // lives in code, never in the Driver Station config.
        //
        // NOTE: shane/consolidate replaces this with a shared
        // RobotHardware.applyCollectionDirections() helper, so that there is one
        // place the collection directions are set. Switch to it when that
        // branch reaches main, and delete this line.
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotTarget = startPosition();
        pivotCommanded = pivotTarget;

        while (opModeInInit()) {
            deflector.update();
            telemetry.addLine("NOT ARMED -- the pivot is not being written to at all.");
            telemetry.addLine("Park the pivot by hand, then read the numbers below.");
            telemetry.addLine();
            telemetry.addData("Pivot would ARM to", "%.3f  (%s)", startPosition(), robotName());
            showPivotFeedback();
            telemetry.addLine();
            telemetry.addLine("Press START, then gamepad1 Y to arm.");
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
            showTelemetry();
            telemetry.update();
        }

        indexer.setPower(0);
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
            pivotTarget = startPosition();
            pivotCommanded = pivotTarget;
            pivot.setPosition(pivotCommanded);
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
            if (rb && !lastRb1) pivotTarget += PIVOT_FINE_STEP;
            if (lb && !lastLb1) pivotTarget -= PIVOT_FINE_STEP;
            if (dl && !lastDl1) pivotTarget = ShooterPivot.POLLEN_POSITION;
            if (dr && !lastDr1) pivotTarget = ShooterPivot.NECTAR_POSITION;

            double jog = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(jog) > 0.05) {
                pivotTarget += jog * PIVOT_COARSE_RATE * dt;
            }

            pivotTarget = Range.clip(pivotTarget,
                    ShooterPivot.MIN_SAFE_POSITION, ShooterPivot.MAX_SAFE_POSITION);

            // Creep toward the target. A wrong preset becomes a slow traverse you
            // can hear coming, not a slam.
            double step = PIVOT_MAX_RATE * dt;
            pivotCommanded += Range.clip(pivotTarget - pivotCommanded, -step, step);
            pivot.setPosition(pivotCommanded);
        }

        lastY1 = y; lastX1 = x; lastA1 = a; lastRb1 = rb; lastLb1 = lb;
        lastDl1 = dl; lastDr1 = dr; lastDu1 = du; lastDd1 = dd;
        lastLs1 = gamepad1.left_stick_button; lastRs1 = gamepad1.right_stick_button;
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
        if (rb && !lastRb2) {
            indexerTargetRpm = Math.min(INDEXER_MAX_RPM, indexerTargetRpm + INDEXER_RPM_STEP);
        }
        if (lb && !lastLb2) {
            indexerTargetRpm = Math.max(0, indexerTargetRpm - INDEXER_RPM_STEP);
        }

        lastY2 = y; lastB2 = b; lastA2 = a; lastRb2 = rb; lastLb2 = lb;
        lastDu2 = du; lastDd2 = dd;
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
        telemetry.addData("Commanded", "%.3f   (target %.3f)", pivotCommanded, pivotTarget);
        telemetry.addData("Presets", "pollen %.3f   nectar %.3f",
                ShooterPivot.POLLEN_POSITION, ShooterPivot.NECTAR_POSITION);
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
        return WhichRobot.get() == WhichRobot.Robot.R5618 ? PIVOT_START_5618 : PIVOT_START_6494;
    }

    private static String robotName() {
        return WhichRobot.number();
    }
}
