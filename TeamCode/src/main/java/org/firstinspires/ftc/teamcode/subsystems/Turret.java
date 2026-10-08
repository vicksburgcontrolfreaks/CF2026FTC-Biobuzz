package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret subsystem: a continuous-rotation servo turning a 30T drive gear into a
 * 69T turret gear (2.3:1 reduction), with a REV Through Bore encoder for position
 * and a magnetic limit switch for homing.
 *
 * A CR servo takes a POWER, not a position -- it has no idea where it is. So this
 * class closes the loop itself: read the encoder, compare to the target angle,
 * and pick a power. That also means the turret does not know where it is pointed
 * until it has been homed, which is why there is a state machine here instead of
 * just a setAngle() method.
 *
 * Driver Station config names this expects:
 *   "turret"        - CRServo (Continuous Rotation Servo) on a servo port
 *   "turretEncoder" - any motor type, on the motor port that the Through Bore
 *                     encoder cable plugs into. NEVER set power on it; we only
 *                     ever read getCurrentPosition().
 *   "turretHome"    - REV Touch Sensor (the magnetic limit switch), digital port
 *
 * Call init() once, then update() every loop. Nothing moves unless update() runs.
 */
@Configurable
public class Turret {

    // ---- Geometry -----------------------------------------------------------
    // Ratio check: the servo must turn 69/30 = 2.3 revs for one full turret rev.
    public static double DRIVE_GEAR_TEETH = 30.0;
    public static double TURRET_GEAR_TEETH = 69.0;

    // MEASURE THIS ON THE ROBOT -- do not trust the starting value.
    // A REV Through Bore Encoder is 8192 ticks per revolution of its own shaft.
    //   Encoder mounted on the TURRET axis:  8192 / 360       = 22.76 ticks/deg
    //   Encoder mounted on the SERVO shaft:  8192 * 2.3 / 360 = 52.33 ticks/deg
    // TurretTest has a "zero reference" button so you can turn the turret a
    // measured 90 degrees and read the real number straight off telemetry.
    public static double TICKS_PER_DEGREE = 22.76;

    // Set to -1 if positive servo power makes the reported angle go DOWN.
    // You will find this out in the first 10 seconds of testing.
    public static double ENCODER_DIRECTION = 1.0;

    // ---- Travel limits ------------------------------------------------------
    // 210 degrees of total travel, centered so 0 = straight ahead.
    public static double MIN_ANGLE = -105.0;
    public static double MAX_ANGLE = 105.0;

    // ---- Homing -------------------------------------------------------------
    // The turret angle where the magnet switch sits, and which way to drive to
    // find it. ASSUMPTION: the magnet is at the negative end of travel. If it is
    // at the other end, flip the sign on both of these.
    public static double HOME_SWITCH_ANGLE = -105.0;
    public static double HOMING_DIRECTION = -1.0;
    public static double HOMING_POWER = 0.25;
    public static double HOMING_TIMEOUT_SEC = 6.0;

    // After homing, drive here so the turret is not resting against the stop.
    public static double PARK_ANGLE = 0.0;

    // ---- Closed loop --------------------------------------------------------
    public static double kP = 0.010;          // power per degree of error
    public static double MAX_POWER = 0.55;
    public static double MIN_POWER = 0.12;    // deadband: below this the servo just buzzes
    public static double TOLERANCE_DEG = 2.0; // close enough -- stop pushing

    // ---- Stall detection ----------------------------------------------------
    // If we are commanding power but the encoder is not moving, we have hit
    // something. Give up instead of grinding the gears.
    public static double STALL_TICKS = 15.0;
    public static double STALL_TIME_SEC = 0.35;

    /** NOT_HOMED: angle unknown, soft limits off. READY: angle known, limits on. */
    public enum State { NOT_HOMED, HOMING, READY, FAILED }

    /** HOLD runs the closed loop to targetAngle. JOG passes raw power through. */
    public enum ControlMode { HOLD, JOG }

    private CRServo servo;
    private DcMotorEx encoder;
    private TouchSensor homeSwitch;

    private State state = State.NOT_HOMED;
    private ControlMode controlMode = ControlMode.JOG;

    private double homeTickOffset = 0.0;   // encoder reading taken at referenceAngle
    private double referenceAngle = 0.0;   // the angle that offset corresponds to
    private double targetAngle = 0.0;
    private double rawPower = 0.0;         // what JOG mode sends
    private double lastAppliedPower = 0.0;
    private String statusNote = "init";

    private final ElapsedTime homingTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private int lastStallTicks = 0;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        homeSwitch = hardwareMap.get(TouchSensor.class, "turretHome");

        servo.setPower(0.0);

        // Treat wherever we booted up as angle 0 until homing proves otherwise.
        // Soft limits stay OFF while NOT_HOMED, because this number is a guess.
        homeTickOffset = encoder.getCurrentPosition();
        referenceAngle = 0.0;
        state = State.NOT_HOMED;
        statusNote = "not homed";
    }

    // ---- Commands -----------------------------------------------------------

    /** Begin homing. It finishes inside update() over the next few seconds. */
    public void startHoming() {
        state = State.HOMING;
        controlMode = ControlMode.HOLD;
        homingTimer.reset();
        resetStallWatch();
        statusNote = "homing";
    }

    /** Closed-loop target in turret degrees. Clamped to the soft limits once homed. */
    public void setTargetAngle(double degrees) {
        targetAngle = isHomed() ? Range.clip(degrees, MIN_ANGLE, MAX_ANGLE) : degrees;
    }

    /** Raw power for JOG mode, -1..1. Soft limits still apply once homed. */
    public void setRawPower(double power) {
        rawPower = Range.clip(power, -1.0, 1.0);
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
        if (mode == ControlMode.HOLD) {
            targetAngle = getAngle();   // do not lurch: hold where we are right now
        }
    }

    /**
     * Declare "the turret is physically at this angle right now" without homing.
     * This is for measuring TICKS_PER_DEGREE during testing, not for match code.
     */
    public void resetReferenceHere(double assumedAngle) {
        homeTickOffset = encoder.getCurrentPosition();
        referenceAngle = assumedAngle;
    }

    /** Run every loop. This is the only place the servo is actually written to. */
    public void update() {
        double power;

        switch (state) {
            case HOMING:
                power = updateHoming();
                break;

            case READY:
                power = (controlMode == ControlMode.JOG) ? rawPower : closedLoopPower();
                power = applySoftLimits(power);
                break;

            case NOT_HOMED:
                // Jogging is allowed so you can find the magnet and measure the
                // encoder, but there are no soft limits to protect you. Go slow.
                power = (controlMode == ControlMode.JOG) ? rawPower : 0.0;
                break;

            case FAILED:
            default:
                power = 0.0;
                break;
        }

        lastAppliedPower = Range.clip(power, -MAX_POWER, MAX_POWER);
        servo.setPower(lastAppliedPower);
    }

    /** Cut power immediately and hold wherever we ended up. */
    public void stop() {
        rawPower = 0.0;
        lastAppliedPower = 0.0;
        if (servo != null) {
            servo.setPower(0.0);
        }
    }

    // ---- Internals ----------------------------------------------------------

    private double updateHoming() {
        if (homeSwitch.isPressed()) {
            // Found it. This encoder reading IS the switch angle, by definition.
            homeTickOffset = encoder.getCurrentPosition();
            referenceAngle = HOME_SWITCH_ANGLE;
            state = State.READY;
            controlMode = ControlMode.HOLD;
            targetAngle = Range.clip(PARK_ANGLE, MIN_ANGLE, MAX_ANGLE);
            statusNote = "homed, parking";
            return 0.0;
        }

        if (homingTimer.seconds() > HOMING_TIMEOUT_SEC) {
            state = State.FAILED;
            statusNote = "HOMING TIMED OUT - never saw the magnet";
            return 0.0;
        }

        if (isStalled()) {
            state = State.FAILED;
            statusNote = "HOMING STALLED - hit a hard stop, check HOMING_DIRECTION";
            return 0.0;
        }

        return HOMING_DIRECTION * HOMING_POWER;
    }

    private double closedLoopPower() {
        double error = targetAngle - getAngle();
        if (Math.abs(error) <= TOLERANCE_DEG) {
            statusNote = "on target";
            return 0.0;
        }
        statusNote = "seeking";

        double power = kP * error;
        // Deadband compensation: below roughly 0.1 power a CR servo just hums,
        // burns current, and never actually turns the turret.
        if (Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }
        return power;
    }

    /** Refuse to drive further past a limit, but always allow coming back. */
    private double applySoftLimits(double power) {
        double angle = getAngle();
        if (angle >= MAX_ANGLE && power > 0.0) {
            statusNote = "at MAX limit";
            return 0.0;
        }
        if (angle <= MIN_ANGLE && power < 0.0) {
            statusNote = "at MIN limit";
            return 0.0;
        }
        return power;
    }

    private void resetStallWatch() {
        stallTimer.reset();
        lastStallTicks = encoder.getCurrentPosition();
    }

    private boolean isStalled() {
        int now = encoder.getCurrentPosition();
        if (Math.abs(now - lastStallTicks) > STALL_TICKS) {
            resetStallWatch();
            return false;
        }
        return stallTimer.seconds() > STALL_TIME_SEC;
    }

    // ---- Readouts (for telemetry) -------------------------------------------

    /** Turret angle in degrees. Only meaningful once isHomed() is true. */
    public double getAngle() {
        double ticks = (encoder.getCurrentPosition() - homeTickOffset) * ENCODER_DIRECTION;
        return referenceAngle + ticks / TICKS_PER_DEGREE;
    }

    public int getRawTicks() {
        return encoder.getCurrentPosition();
    }

    public int getTicksSinceReference() {
        return (int) (encoder.getCurrentPosition() - homeTickOffset);
    }

    public boolean isHomeSwitchPressed() {
        return homeSwitch.isPressed();
    }

    public boolean isHomed() {
        return state == State.READY;
    }

    public boolean isOnTarget() {
        return isHomed() && Math.abs(targetAngle - getAngle()) <= TOLERANCE_DEG;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getAppliedPower() {
        return lastAppliedPower;
    }

    public State getState() {
        return state;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public String getStatusNote() {
        return statusNote;
    }

    /** Servo revs per turret rev -- a sanity check on the gearing numbers. */
    public static double gearRatio() {
        return TURRET_GEAR_TEETH / DRIVE_GEAR_TEETH;
    }
}
