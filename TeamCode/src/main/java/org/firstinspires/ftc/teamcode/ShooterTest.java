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

/**
 * Bring-up op mode for the two shooter flywheels, and nothing else.
 *
 * STANDALONE. The active config needs exactly two devices:
 *   "shooterLeft"   flywheel, EH2
 *   "shooterRight"  flywheel, CH2
 * No turret, no camera, no pinpoint, no collection system. So this runs with
 * most of the robot unplugged and it cannot die at init on a device you were
 * not testing.
 *
 * CONTROLS (gamepad1)
 *   A          shooter ON / OFF
 *   X          toggle POWER mode / VELOCITY mode
 *   R trigger  POWER mode: open-loop power, analog
 *   R/L bumper VELOCITY mode: target RPM +/- RPM_STEP
 *   Y          reset the peak-RPM hold, to measure again
 *   B          panic: shooter off, target back to 0
 *
 * WHY TWO MODES, AND WHICH ONE YOU WANT
 *
 * POWER mode is open loop -- it hands the motor a voltage and nobody checks the
 * result. Hold the trigger at full and "Peak flywheel RPM" is the real, honest
 * ceiling of this shooter. That is the number nobody has measured yet, and
 * every RPM target in the codebase should sit below it.
 *
 * VELOCITY mode is closed loop -- you ask for an RPM and the hub's PIDF chases
 * it. Useful once you know the ceiling, misleading before: ask for more than
 * the motor can deliver and the PIDF saturates, which looks exactly like bad
 * tuning. That is what the "BELOW TARGET" line is for. If you see it, lower the
 * target before you touch a single PIDF value.
 *
 * WHAT TO WATCH FOR
 *
 * 1. DO THEY THROW, OR FIGHT? The wheels face each other, so shooterRight is
 *    reversed in code and both then take the same commanded value. If they
 *    fight each other instead of throwing a ball, that reversal is wrong -- fix
 *    it in RobotHardware, not here, or the two files disagree forever.
 *
 * 2. "L-R gap" SHOULD STAY SMALL. A steady gap under load is normal. A large
 *    gap with nothing touching the wheels is not, and the two motors living on
 *    different hubs (EH2 and CH2) is the first suspect -- each hub runs its own
 *    velocity PIDF locally.
 *
 * 3. THE GEAR RATIO IS 3:1 AND IT IS THE MOTOR'S OWN GEARBOX. Every RPM here is
 *    flywheel RPM, converted through RobotHardware.SHOOTER_GEAR_RATIO. If these
 *    turn out to be REV UltraPlanetary cartridges, their printed "3:1" is
 *    really 2.89:1 -- change it in RobotHardware and every op mode follows.
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

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

    private DcMotorEx shooterLeft, shooterRight;

    private ShooterMode shooterMode = ShooterMode.POWER;
    private boolean shooterOn = false;
    private double targetRpm = 0.0;
    private double peakRpm = 0.0;

    private boolean aPrev, xPrev, bPrev, yPrev, rbPrev, lbPrev;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
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

        telemetry.addLine("SHOOTER TEST");
        telemetry.addLine("A = on/off | X = power/velocity | RT = power | bumpers = RPM");
        telemetry.addLine("Y = reset peak | B = panic off");
        telemetry.addLine("");
        telemetry.addLine("Start in POWER mode, hold RT full, read 'Peak flywheel RPM'.");
        telemetry.addLine("That peak is the real ceiling. Tell it to Claude to cap the rest.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleInput();
            driveShooter();

            double leftRpm = flywheelRpm(shooterLeft.getVelocity());
            double rightRpm = flywheelRpm(shooterRight.getVelocity());
            peakRpm = Math.max(peakRpm, Math.max(leftRpm, rightRpm));

            showTelemetry(leftRpm, rightRpm);
        }

        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    private void handleInput() {
        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (a && !aPrev) {
            shooterOn = !shooterOn;
        }
        if (x && !xPrev) {
            shooterMode = (shooterMode == ShooterMode.POWER)
                    ? ShooterMode.VELOCITY
                    : ShooterMode.POWER;
            applyShooterMode();
        }
        if (b && !bPrev) {
            shooterOn = false;
            targetRpm = 0.0;
        }
        if (y && !yPrev) {
            peakRpm = 0.0;
        }
        if (rb && !rbPrev) {
            targetRpm = Math.min(targetRpm + RPM_STEP, MAX_TARGET_RPM);
        }
        if (lb && !lbPrev) {
            targetRpm = Math.max(targetRpm - RPM_STEP, 0.0);
        }

        aPrev = a;
        xPrev = x;
        bPrev = b;
        yPrev = y;
        rbPrev = rb;
        lbPrev = lb;
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
            double power = gamepad1.right_trigger;
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

    private void showTelemetry(double leftRpm, double rightRpm) {
        telemetryM.addData("Shooter", shooterOn ? "ON" : "off");
        telemetryM.addData("Mode", shooterMode);
        if (shooterMode == ShooterMode.POWER) {
            telemetryM.addData("Commanded power", shooterOn ? gamepad1.right_trigger : 0.0);
        } else {
            telemetryM.addData("Target RPM", targetRpm);
        }
        telemetryM.addLine("");
        telemetryM.addData("Actual L RPM", leftRpm);
        telemetryM.addData("Actual R RPM", rightRpm);
        telemetryM.addData("L-R gap", Math.abs(leftRpm - rightRpm));
        telemetryM.addData("Peak flywheel RPM", peakRpm);
        telemetryM.addLine("");
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

        telemetryM.update(telemetry);
    }
}
