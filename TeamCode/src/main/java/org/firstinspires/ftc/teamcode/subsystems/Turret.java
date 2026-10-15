package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 *   "turret"     - CRServo (Continuous Rotation Servo) on a servo port
 *   "turretHome" - REV Touch Sensor (the magnetic limit switch), digital port
 *   "rf"         - the RIGHT FRONT DRIVE MOTOR. The turret encoder is plugged into
 *                  that motor port's spare encoder connector, so there is no
 *                  separate encoder device to configure. Read the warning below.
 *
 * SHARING THE "rf" MOTOR PORT
 * The drivetrain owns "rf". This class only ever READS its encoder -- it never
 * sets power, direction, run mode, or resets the encoder on it. Two rules keep
 * that safe, and breaking either one breaks the turret:
 *
 *   1. "rf" must stay in RUN_WITHOUT_ENCODER (the default). If anything puts it
 *      in RUN_USING_ENCODER or RUN_TO_POSITION, the hub's velocity PID would try
 *      to regulate the drive wheel using the TURRET's position. Nothing in this
 *      repo does that today, and Pedro Pathing's Mecanum class does not either.
 *   2. Nobody calls STOP_AND_RESET_ENCODER on "rf", which would zero the turret's
 *      position mid-match.
 *
 * Direction is handled for us: the SDK negates getCurrentPosition() when a motor
 * is REVERSE (DcMotorImpl.adjustPosition), and this repo sets rf's direction
 * differently in different op modes -- Pedro's Constants says REVERSE while
 * RobotHardware says FORWARD. readRawTicks() divides that out so turret angles
 * mean the same thing everywhere, and ENCODER_DIRECTION stays the only sign knob.
 *
 * Call init() once, then update() every loop. Nothing moves unless update() runs.
 */
@Configurable
public class Turret {

    // ---- Geometry -----------------------------------------------------------
    // Ratio check: the servo must turn 69/30 = 2.3 revs for one full turret rev.
    public static double DRIVE_GEAR_TEETH = 20.0;
    public static double TURRET_GEAR_TEETH = 69.0;

    // DERIVED FROM GEAR TEETH, not from a sweep -- this one is exact:
    //   REV Through Bore V2 (REV-11-3174)        = 8192 counts/rev
    //   encoder gear 20T on the 69T ring         = 3.45 : 1
    //   8192 * 3.45 / 360                        = 78.51 ticks/deg
    //
    // Every term is an integer from a spec sheet or a tooth count, so there is no
    // measurement error in this number at all. Confirmed in CAD 2026-10-15: both
    // small gears are OD 44 mm and the ring is OD 142 mm, which at module 2 is
    // 20T (44 = 2 * (20+2)) and 69T (142 = 2 * (69+2)).
    //
    // The encoder does NOT ride the servo's pinion -- it has its own 20T gear
    // meshing the same ring, so it happens to share the 3.45:1 ratio.
    //
    // MEASURED 2026-10-13 with TurretRangeTest, pushing by hand to both hard stops:
    //   stop A -9617 ticks, magnet -124, stop B +12832  ->  span 22449 ticks
    // Against 78.51 ticks/deg that span is 286 deg of travel. Two earlier values
    // were wrong: 106.90 (span / a guessed 210 deg) and 95.73 (span / a CAD angle
    // read as 234.5, later 232). A CAD angle of 232 deg would require the span to
    // be 18214 ticks; the encoder counted 22449. Encoders do not miscount by 23%,
    // and a tape-and-count sweep gave ~2.6 encoder turns against the 2.74 that
    // 22449/8192 implies -- the 232 deg reading predicts only 2.22 turns.
    public static double TICKS_PER_DEGREE = 78.51;

    // Set to -1 if positive servo power makes the reported angle go DOWN.
    // You will find this out in the first 10 seconds of testing. This is the ONLY
    // place turret sign should be adjusted -- do not "fix" it by changing rf's
    // direction, which belongs to the drivetrain.
    public static double ENCODER_DIRECTION = 1.0;

    // The drive motor whose spare encoder port the turret encoder is wired into.
    private static final String ENCODER_MOTOR_NAME = "rf";

    // ---- Travel limits ------------------------------------------------------
    // Travel works out to 286 degrees, so the hard stops sit at -143.0 and +143.0
    // either side of center. These limits are pulled in 3 degrees from the stops so
    // the turret never rests against one; widen toward 143 if the extra reach is
    // needed, but do not exceed it.
    //
    // Because ticks-per-degree and these limits both trace back to the same 22449
    // tick sweep, the limits land on the real stops even if the degree scale is
    // later revised -- a scale error shifts the labels, not where the turret stops.
    public static double MIN_ANGLE = -140.0;
    public static double MAX_ANGLE = 140.0;

    // ---- Homing -------------------------------------------------------------
    // MEASURED: the magnet sits at -22.1 degrees -- near the MIDDLE of travel,
    // 120.9 degrees from one hard stop and 165.0 from the other. That is why homing
    // searches both directions instead of driving one fixed way: from a cold start
    // the turret may be on either side of the magnet, and roughly 42% of starting
    // positions would otherwise drive away from it into a hard stop.
    //
    // This is the CENTER of the magnet's trigger band (about 530 ticks, so 6.8 deg),
    // measured by sweeping through it. Homing trips on whichever edge it reaches
    // first, which is repeatable because the search always approaches from the same
    // side.
    public static double HOME_SWITCH_ANGLE = -22.06;

    // Which way to search FIRST. The magnet is just below center, so searching
    // negative finds it quickly from most positions. If the first direction stalls
    // against a stop, homing reverses automatically.
    public static double HOMING_DIRECTION = -1.0;
    public static double HOMING_POWER = 0.25;

    // Per-direction budget, not a total. Worst case is a full 165 degrees of travel
    // to reach the magnet, so 6.0 is probably too short -- expect to raise it once
    // the turret's actual speed at HOMING_POWER is known.
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
    /** Shared with the drivetrain. READ ONLY -- see the class comment. */
    private DcMotorEx encoderMotor;
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

    // Homing search state. The magnet is mid-travel, so one direction may run into a
    // hard stop before reaching it; when that happens the search flips once.
    private double homingSearchDir = HOMING_DIRECTION;
    private boolean homingReversed = false;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, hardwareMap.get(DcMotorEx.class, ENCODER_MOTOR_NAME));
    }

    /**
     * Same as init(hardwareMap) but reuses a drive motor handle the caller already
     * has. Use this from RobotHardware so there is one rf object, not two.
     */
    public void init(HardwareMap hardwareMap, DcMotorEx sharedEncoderMotor) {
        servo = hardwareMap.get(CRServo.class, "turret");
        encoderMotor = sharedEncoderMotor;
        homeSwitch = hardwareMap.get(TouchSensor.class, "turretHome");

        servo.setPower(0.0);

        // Treat wherever we booted up as angle 0 until homing proves otherwise.
        // Soft limits stay OFF while NOT_HOMED, because this number is a guess.
        homeTickOffset = readRawTicks();
        referenceAngle = 0.0;
        state = State.NOT_HOMED;
        statusNote = "not homed";
    }

    // ---- Commands -----------------------------------------------------------

    /** Begin homing. It finishes inside update() over the next few seconds. */
    public void startHoming() {
        state = State.HOMING;
        controlMode = ControlMode.HOLD;
        homingSearchDir = HOMING_DIRECTION;
        homingReversed = false;
        homingTimer.reset();
        resetStallWatch();
        statusNote = "homing: searching " + (homingSearchDir < 0 ? "-" : "+");
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
        homeTickOffset = readRawTicks();
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
            homeTickOffset = readRawTicks();
            referenceAngle = HOME_SWITCH_ANGLE;
            state = State.READY;
            controlMode = ControlMode.HOLD;
            targetAngle = Range.clip(PARK_ANGLE, MIN_ANGLE, MAX_ANGLE);
            statusNote = "homed, parking";
            return 0.0;
        }

        // Either the search ran out of time or it ran into a hard stop. Both mean
        // "the magnet is not this way", so flip once and try the other direction.
        boolean timedOut = homingTimer.seconds() > HOMING_TIMEOUT_SEC;
        boolean hitSomething = isStalled();

        if (timedOut || hitSomething) {
            if (!homingReversed) {
                homingReversed = true;
                homingSearchDir = -homingSearchDir;
                homingTimer.reset();
                resetStallWatch();
                statusNote = (hitSomething ? "hit a stop" : "timed out")
                        + ", reversing search";
                return 0.0;   // pause a loop before driving the other way
            }

            state = State.FAILED;
            statusNote = hitSomething
                    ? "HOMING FAILED - hit stops both ways, never saw the magnet"
                    : "HOMING FAILED - timed out both ways; raise HOMING_TIMEOUT_SEC";
            return 0.0;
        }

        return homingSearchDir * HOMING_POWER;
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

    /**
     * Encoder ticks with the drive motor's direction setting cancelled out.
     *
     * The SDK runs getCurrentPosition() through DcMotorImpl.adjustPosition(), which
     * negates the value when the motor is REVERSE. rf's direction is the
     * drivetrain's business and is not consistent across op modes, so undoing it
     * here keeps a given turret angle meaning the same thing in TeleOp and in auto.
     * Re-read every call rather than cached at init, in case something changes
     * rf's direction while the op mode is running.
     */
    private int readRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        if (encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            ticks = -ticks;
        }
        return ticks;
    }

    private void resetStallWatch() {
        stallTimer.reset();
        lastStallTicks = readRawTicks();
    }

    private boolean isStalled() {
        int now = readRawTicks();
        if (Math.abs(now - lastStallTicks) > STALL_TICKS) {
            resetStallWatch();
            return false;
        }
        return stallTimer.seconds() > STALL_TIME_SEC;
    }

    // ---- Readouts (for telemetry) -------------------------------------------

    /** Turret angle in degrees. Only meaningful once isHomed() is true. */
    public double getAngle() {
        double ticks = (readRawTicks() - homeTickOffset) * ENCODER_DIRECTION;
        return referenceAngle + ticks / TICKS_PER_DEGREE;
    }

    public int getRawTicks() {
        return readRawTicks();
    }

    public int getTicksSinceReference() {
        return (int) (readRawTicks() - homeTickOffset);
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
