package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Bring-up op mode for the shooter flywheels and the turret, together, with no
 * vision involved. Homing and moving the turret, and spinning the two flywheels.
 *
 * STANDALONE ON PURPOSE. The active config only needs five devices:
 *   "turret"       CR servo
 *   "turretHome"   magnetic limit switch
 *   "lf"           left front drive motor -- carries the turret's encoder
 *   "shooterLeft"  flywheel
 *   "shooterRight" flywheel
 * No camera, no pinpoint, no pivot, no collection system. So this runs with the
 * rest of the robot unplugged, and it cannot die at init on a device you were
 * not testing.
 *
 * THIS IS NOT ShootAprilTag. That class aims by driving the drive wheels, from
 * before the turret existed, and it has unfixed PID bugs. Nothing here touches
 * the drivetrain.
 *
 * TURRET CONTROLS (gamepad1) -- identical to Turret Test, on purpose
 *   Triggers   JOG: right = positive power, left = negative power
 *   Y          toggle JOG / HOLD (closed loop)
 *   X          start homing
 *   B          "the turret is at 0 right now" (zeroes the encoder reference)
 *   D-pad L/R  HOLD mode: nudge target by NUDGE_DEG
 *   A          HOLD mode: send target to 0 (straight ahead)
 *
 * SHOOTER CONTROLS (gamepad2) -- a different gamepad so a turret jog can never
 * spin up a flywheel by accident
 *   A          shooter ON / OFF
 *   X          toggle POWER mode / VELOCITY mode
 *   R trigger  POWER mode: open-loop power, analog
 *   R/L bumper VELOCITY mode: target RPM +/- RPM_STEP
 *   B          panic: shooter off, target back to 0
 *
 * WHY TWO SHOOTER MODES, AND WHICH ONE YOU WANT
 *
 * POWER mode is open loop -- it hands the motor a voltage and nobody checks the
 * result. Hold the trigger at full and "Peak flywheel RPM" is the real, honest
 * ceiling of this shooter. That is the number nobody has measured yet, and
 * every RPM target should be set below it.
 *
 * VELOCITY mode is closed loop -- you ask for an RPM and the hub's PIDF chases
 * it. Useful once you know the ceiling, useless before: ask for more than the
 * motor can deliver and the PIDF saturates, which looks exactly like bad tuning.
 * That is why "BELOW TARGET" is on the telemetry. If you see it, lower the
 * target before you touch a single PIDF value.
 *
 * WHAT TO WATCH FOR
 *
 * 1. TURRET SIGN, FIRST, BEFORE HOMING. Jog with the right trigger and watch
 *    "Angle (deg)". Positive power must make the angle go UP. If it goes DOWN,
 *    set Turret.ENCODER_DIRECTION to -1.0 in Panels. That is the only place
 *    turret sign should ever be adjusted, and homing cannot work until it is
 *    right -- a wrong sign makes the closed loop run AWAY from its target and
 *    drive into the hard stop. The encoder cable moved from rf to lf on
 *    2026-10-27, so this needs re-checking even though it was right before.
 *
 * 2. THE TURRET STARTS NOT_HOMED, WHICH MEANS NO SOFT LIMITS. Jog gently, keep
 *    a finger near the stop. HOLD mode does nothing at all until homing
 *    succeeds, by design -- an un-homed angle is a guess, so the subsystem
 *    refuses to chase a target built on it.
 *
 * 3. BOTH FLYWHEELS SHOULD READ THE SAME RPM. "L-R gap" is the difference. The
 *    wheels face each other with shooterRight reversed, so if they fight
 *    instead of throwing, that reversal is wrong. A steady gap under load is
 *    normal; a large gap with nothing touching the wheels is not, and the two
 *    motors being on different hubs (EH2 and CH2) is the first suspect.
 *
 * 4. THE SHOOTER IS FORCED OFF WHILE THE TURRET IS HOMING. Homing wants your
 *    hand near the turret. Live flywheels do not. Not a bug.
 */
@Configurable
@TeleOp(name = "Shooter + Turret Test", group = "Test")
public class ShooterTurretTest extends LinearOpMode {

    // ---- Turret tunables (same names and defaults as Turret Test) ----------
    public static double JOG_SCALE = 0.35;   // trigger 1.0 -> this much power
    public static double NUDGE_DEG = 10.0;

    // ---- Shooter tunables ---------------------------------------------------
    public static double RPM_STEP = 100.0;
    /**
     * Ceiling on what VELOCITY mode will let you ask for. An HD Hex free-speeds
     * near 6000 RPM at the motor, so through the 3:1 the flywheel cannot beat
     * about 2000 even with nothing touching it. Starts there deliberately --
     * measure the real number in POWER mode, then bring this down to about 85%
     * of it so the PIDF has headroom to actually regulate.
     */
    public static double MAX_TARGET_RPM = 2000.0;

    private static final double TICKS_PER_MOTOR_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double SHOOTER_RATIO = RobotHardware.SHOOTER_GEAR_RATIO;

    private enum ShooterMode { POWER, VELOCITY }

    private final Turret turret = new Turret();
    private DcMotorEx shooterLeft, shooterRight;

    private ShooterMode shooterMode = ShooterMode.POWER;
    private boolean shooterOn = false;
    private double targetRpm = 0.0;
    private double peakRpm = 0.0;

    private boolean yPrev, xPrev, bPrev, aPrev, dpadLeftPrev, dpadRightPrev;
    private boolean g2aPrev, g2xPrev, g2bPrev, g2rbPrev, g2lbPrev;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        // One-arg init: the Turret grabs "lf" itself. Safe here because nothing
        // in this op mode drives the drivetrain, so there is no second owner.
        turret.init(hardwareMap);

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        // Mirrors RobotHardware: the wheels face each other, so one side is
        // reversed and both then take the same commanded value.
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Coast, never brake. Braking a loaded flywheel punishes the gearbox.
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyShooterMode();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(turret);

        telemetry.addLine("SHOOTER + TURRET TEST");
        telemetry.addLine("GP1 turret: triggers = jog | Y = jog/hold | X = home | B = call this 0");
        telemetry.addLine("GP2 shooter: A = on/off | X = power/velocity | RT or bumpers");
        telemetry.addLine("");
        telemetry.addLine("CHECK THE TURRET SIGN FIRST: jog with RT, 'Angle' must go UP.");
        telemetry.addLine("NOT HOMED on start: soft limits are OFF. Jog gently.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleTurretInput();
            handleShooterInput();

            turret.update();

            // Safety interlock: homing wants hands near the turret, so the
            // flywheels do not get to be live at the same time.
            boolean homingNow = turret.getState() == Turret.State.HOMING;
            if (homingNow) {
                shooterOn = false;
            }
            driveShooter();

            double leftRpm = flywheelRpm(shooterLeft.getVelocity());
            double rightRpm = flywheelRpm(shooterRight.getVelocity());
            peakRpm = Math.max(peakRpm, Math.max(leftRpm, rightRpm));

            showTelemetry(leftRpm, rightRpm, homingNow);
        }

        turret.stop();
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    // ---- Turret -------------------------------------------------------------

    private void handleTurretInput() {
        boolean y = gamepad1.y;
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean a = gamepad1.a;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (y && !yPrev) {
            turret.setControlMode(turret.getControlMode() == Turret.ControlMode.JOG
                    ? Turret.ControlMode.HOLD
                    : Turret.ControlMode.JOG);
        }
        if (x && !xPrev) {
            turret.startHoming();
        }
        if (b && !bPrev) {
            // Line the turret up straight ahead by eye, then press B. The angle
            // readout is real after that, so ticks per degree can be measured.
            turret.resetReferenceHere(0.0);
        }
        if (a && !aPrev) {
            turret.setTargetAngle(0.0);
        }
        if (dpadRight && !dpadRightPrev) {
            turret.setTargetAngle(turret.getTargetAngle() + NUDGE_DEG);
        }
        if (dpadLeft && !dpadLeftPrev) {
            turret.setTargetAngle(turret.getTargetAngle() - NUDGE_DEG);
        }

        yPrev = y;
        xPrev = x;
        bPrev = b;
        aPrev = a;
        dpadLeftPrev = dpadLeft;
        dpadRightPrev = dpadRight;

        // Deliberately NOT deadband-compensated, so you can feel exactly where
        // the servo starts to move and set MIN_POWER from it.
        turret.setRawPower((gamepad1.right_trigger - gamepad1.left_trigger) * JOG_SCALE);
    }

    // ---- Shooter ------------------------------------------------------------

    private void handleShooterInput() {
        boolean a = gamepad2.a;
        boolean x = gamepad2.x;
        boolean b = gamepad2.b;
        boolean rb = gamepad2.right_bumper;
        boolean lb = gamepad2.left_bumper;

        if (a && !g2aPrev) {
            shooterOn = !shooterOn;
        }
        if (x && !g2xPrev) {
            shooterMode = (shooterMode == ShooterMode.POWER)
                    ? ShooterMode.VELOCITY
                    : ShooterMode.POWER;
            applyShooterMode();
        }
        if (b && !g2bPrev) {
            shooterOn = false;
            targetRpm = 0.0;
        }
        if (rb && !g2rbPrev) {
            targetRpm = Math.min(targetRpm + RPM_STEP, MAX_TARGET_RPM);
        }
        if (lb && !g2lbPrev) {
            targetRpm = Math.max(targetRpm - RPM_STEP, 0.0);
        }

        g2aPrev = a;
        g2xPrev = x;
        g2bPrev = b;
        g2rbPrev = rb;
        g2lbPrev = lb;
    }

    /**
     * POWER mode has to be genuinely open loop for the peak reading to mean
     * anything, so the run mode is set explicitly. setVelocity() puts the motors
     * into RUN_USING_ENCODER on its own, so VELOCITY mode needs nothing here.
     */
    private void applyShooterMode() {
        if (shooterMode == ShooterMode.POWER) {
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void driveShooter() {
        if (!shooterOn) {
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
            return;
        }

        if (shooterMode == ShooterMode.POWER) {
            double power = gamepad2.right_trigger;
            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        } else {
            double ticksPerSec = targetRpm * TICKS_PER_MOTOR_REV * SHOOTER_RATIO / 60.0;
            shooterLeft.setVelocity(ticksPerSec);
            shooterRight.setVelocity(ticksPerSec);
        }
    }

    /** Motor encoder ticks/sec -> flywheel RPM, through the 3:1 gearbox. */
    private double flywheelRpm(double motorTicksPerSec) {
        return (motorTicksPerSec / (TICKS_PER_MOTOR_REV * SHOOTER_RATIO)) * 60.0;
    }

    // ---- Telemetry ----------------------------------------------------------

    private void showTelemetry(double leftRpm, double rightRpm, boolean homingNow) {
        telemetryM.addLine("-- TURRET (gamepad1) --");
        telemetryM.addData("State", turret.getState());
        telemetryM.addData("Mode", turret.getControlMode());
        telemetryM.addData("Note", turret.getStatusNote());
        telemetryM.addData("Home switch", turret.isHomeSwitchPressed() ? "PRESSED" : "open");
        telemetryM.addData("Angle (deg)", turret.getAngle());
        telemetryM.addData("Target (deg)", turret.getTargetAngle());
        telemetryM.addData("On target", turret.isOnTarget());
        telemetryM.addData("Raw ticks", turret.getRawTicks());
        telemetryM.addData("Commanded power", turret.getAppliedPower());
        if (!turret.isHomed()) {
            telemetryM.addLine("NOT HOMED - angle is a guess, soft limits OFF");
        }

        telemetryM.addLine("");
        telemetryM.addLine("-- SHOOTER (gamepad2) --");
        telemetryM.addData("Shooter", shooterOn ? "ON" : "off");
        telemetryM.addData("Shooter mode", shooterMode);
        if (shooterMode == ShooterMode.POWER) {
            telemetryM.addData("Commanded power", shooterOn ? gamepad2.right_trigger : 0.0);
        } else {
            telemetryM.addData("Target RPM", targetRpm);
        }
        telemetryM.addData("Actual L RPM", leftRpm);
        telemetryM.addData("Actual R RPM", rightRpm);
        telemetryM.addData("L-R gap", Math.abs(leftRpm - rightRpm));
        telemetryM.addData("Peak flywheel RPM", peakRpm);
        telemetryM.addData("Amps L", shooterLeft.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Amps R", shooterRight.getCurrent(CurrentUnit.AMPS));

        // The saturation warning. Only meaningful in closed loop, and only once
        // there is a target worth missing.
        if (shooterOn && shooterMode == ShooterMode.VELOCITY && targetRpm > 0) {
            double worst = Math.min(leftRpm, rightRpm);
            if (worst < targetRpm * 0.9) {
                telemetryM.addLine("BELOW TARGET - lower the target before touching PIDF");
            }
        }
        if (homingNow) {
            telemetryM.addLine("Shooter forced OFF while homing");
        }

        telemetryM.update(telemetry);
    }
}
