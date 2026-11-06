package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The deflector: an Axon MAX MK2 in CONTINUOUS ROTATION mode, with its own
 * analog feedback wire. Sets the ball's departure angle out of the shooter.
 *
 * WHY THIS IS A SUBSYSTEM AND NOT JUST OP MODE CODE
 * Two op modes drive this mechanism -- Prematch parks it, and the shooting
 * sequence aims it -- and the direction sign is the kind of constant that, kept
 * in two places, drifts until one copy drives the servo the wrong way. One copy
 * here.
 *
 * HOW POSITION WORKS, AND THE ONE THING TO UNDERSTAND
 * A continuous servo takes a SPEED, not an angle. The only position source is
 * the feedback wire, and it reads 0 V at 0 degrees up to about 3.3 V at 360,
 * then WRAPS back to 0 V. The deflector's useful travel is several turns
 * (SHOOT_ANGLE_DEG is about 4.7 of them), so:
 *
 *   A SINGLE VOLTAGE READING CANNOT TELL YOU WHERE THE DEFLECTOR IS.
 *
 * It is ambiguous modulo 360. getAngle() counts through the wraps and so means
 * something, but only RELATIVE to the last zeroHere() -- and that count lives in
 * this object, which dies with the op mode.
 *
 * WHICH IS FINE, BECAUSE THE MECHANISM CARRIES THE STATE
 * The match sequence is: Prematch parks the deflector physically down at its
 * zero, and it is left there. Auto then starts, assumes zero -- true by
 * construction, because the thing is sitting at zero -- and drives to
 * SHOOT_ANGLE_DEG. Nothing has to be remembered across op modes, and the robot
 * can be powered off in between.
 *
 * WHEN THAT ASSUMPTION IS WRONG, AND WHAT TO DO
 * If auto never ran, or died partway through its move, TeleOp inherits a
 * deflector at an unknown angle and cannot work out where it is from the wire.
 * That is what startHoming() is for: drive gently toward the zero end and watch
 * the angle: when it stops changing while still being pushed, it is against the
 * hard stop. That is a zero reference recovered from the mechanism itself, with
 * nothing remembered. State is UNKNOWN until either zeroHere() or homing has
 * run, so an op mode can refuse to make big moves before then.
 *
 * NON-BLOCKING ON PURPOSE
 * update() is the ONLY place the servo is written to, and it returns
 * immediately. Call it every loop. A blocking "drive until you arrive" loop
 * would stall the flywheels and the indexer while the deflector moved, which is
 * exactly wrong in a shooting op mode.
 *
 * Driver Station config:
 *   "deflector"    - CRServo, type "Continuous Rotation Servo", CH servo 2
 *   "deflectorPos" - AnalogInput, type "Analog Input", CH analog 1
 *   Both the same on 5618 and 6494 (checked 2026-11-06).
 */
@Configurable
public class Deflector {

    private static final String SERVO_NAME = "deflector";
    private static final String FEEDBACK_NAME = "deflectorPos";

    /**
     * Degrees the feedback wire covers from 0 V to its max. AN ASSUMPTION: 360 is
     * the usual Axon MAX MK2 spec but has not been checked on this mechanism.
     * Every angle here scales with it, so if the measured travel disagrees with
     * the physical travel, this is the number to fix -- not the targets.
     */
    public static double ASSUMED_SPAN_DEG = 360.0;

    /**
     * STARTING GUESS at which way is which: +1 if positive power makes
     * getAngle() increase. Mounting and gearing decide it, so nobody can know it
     * from reading the code -- and a wrong sign means a commanded move drives
     * AWAY from its target for the whole timeout, which in auto is the deflector
     * travelling the wrong way for the entire autonomous period.
     *
     * So this is only the first attempt. The first commanded move checks it
     * against what actually happens and flips if it was wrong; see checkSign().
     * Nothing has to be set by hand, and jogging never cared in the first place
     * because the driver can see which way it went.
     */
    public static double POWER_SIGN = 1.0;

    /** How long to drive before judging whether the error is growing. */
    public static double SIGN_CHECK_SEC = 0.4;

    /**
     * How much the error has to change for the check to call it. Big enough that
     * sensor noise cannot decide the answer, small enough to settle in one look.
     */
    public static double SIGN_CHECK_DEG = 3.0;

    /** Travel limit for everything here. A CR servo will drive into a hard stop. */
    public static double MAX_POWER = 0.30;

    /** Inside APPROACH_DEG of the target, drop to this so it does not coast past. */
    public static double SLOW_POWER = 0.10;
    public static double APPROACH_DEG = 120.0;

    /** Close enough. A coasting CR servo will never land exactly. */
    public static double TOLERANCE_DEG = 5.0;

    /** Parked, legal-to-start position. Zero by definition of the reference. */
    public static double ZERO_ANGLE_DEG = 0.0;

    /** Where auto drives it to shoot from. A placeholder until range testing. */
    public static double SHOOT_ANGLE_DEG = 1700.0;

    /** Give up on a move. Several turns at MAX_POWER is not instant. */
    public static double MOVE_TIMEOUT_SEC = 20.0;

    // ---- Homing: find the zero hard stop without remembering anything --------

    /** Which way the zero hard stop is, in the same sign as getAngle(). */
    public static double HOMING_DIRECTION = -1.0;

    /** Gentle: this pushes into a hard stop by design. */
    public static double HOMING_POWER = 0.15;

    /** Moving less than this, for this long, while still driven = at the stop. */
    public static double HOMING_STALL_DEG = 2.0;
    public static double HOMING_STALL_SEC = 0.35;

    /** Come off the stop so the servo is not left leaning on it. */
    public static double HOMING_BACKOFF_DEG = 10.0;

    public static double HOMING_TIMEOUT_SEC = 25.0;

    /** UNKNOWN: angle means nothing yet. READY: angle is relative to a real zero. */
    public enum State { UNKNOWN, HOMING, READY, FAILED }

    /** HOLD chases targetAngle. JOG passes raw power through, for the triggers. */
    public enum ControlMode { HOLD, JOG }

    private CRServo servo;
    private AnalogInput feedback;

    private State state = State.UNKNOWN;
    private ControlMode mode = ControlMode.JOG;

    private double lastRawAngle;
    private double totalDegrees = 0;

    private double targetAngle = 0;
    private double rawPower = 0;

    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallReferenceAngle = 0;
    private boolean homingBackingOff = false;
    private String failure = null;

    // The sign actually in use. Starts as POWER_SIGN and may flip once, the
    // first time a commanded move shows it was backwards.
    private double effectiveSign = 1.0;
    private boolean signLearned = false;
    private double signCheckError = 0;
    private final ElapsedTime signTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        feedback = hardwareMap.get(AnalogInput.class, FEEDBACK_NAME);
        lastRawAngle = readRawAngle();
        totalDegrees = 0;
        state = State.UNKNOWN;
        mode = ControlMode.JOG;
        rawPower = 0;
        effectiveSign = Math.signum(POWER_SIGN) == 0 ? 1.0 : Math.signum(POWER_SIGN);
        signLearned = false;
    }

    /**
     * Call every loop, before reading any angle. The only place the servo is
     * written to. Returns immediately -- moves finish over following calls.
     */
    public void update() {
        accumulate();

        switch (state) {
            case HOMING:
                updateHoming();
                return;

            case UNKNOWN:
            case READY:
            case FAILED:
            default:
                if (mode == ControlMode.JOG) {
                    servo.setPower(clamp(rawPower, -MAX_POWER, MAX_POWER));
                } else {
                    servo.setPower(holdPower());
                }
        }
    }

    /** Power that chases targetAngle, or 0 once inside tolerance or timed out. */
    private double holdPower() {
        double error = targetAngle - totalDegrees;

        if (Math.abs(error) <= TOLERANCE_DEG) {
            return 0;
        }
        if (moveTimer.seconds() > MOVE_TIMEOUT_SEC) {
            failure = String.format(
                    "deflector move timed out %.0fs short of %.0f deg (at %.0f). "
                            + "Jammed, or the wire stopped reading.",
                    MOVE_TIMEOUT_SEC, targetAngle, totalDegrees);
            return 0;
        }

        checkSign(error);

        double speed = Math.abs(error) > APPROACH_DEG ? MAX_POWER : SLOW_POWER;
        return effectiveSign * Math.signum(error) * speed;
    }

    /**
     * Work out which way is which by watching what happens, instead of asking
     * somebody to measure it and write it down.
     *
     * Drive for SIGN_CHECK_SEC and compare the error to what it was:
     *
     *   error shrank  -> the sign is right, stop checking
     *   error grew    -> we are driving away from the target, so flip it
     *   neither       -> it has barely moved. That is a jam or a slow start, NOT
     *                    evidence about direction, so take a fresh baseline and
     *                    look again rather than guessing.
     *
     * That third case is the reason this compares against a threshold rather
     * than just the sign of the change: a stuck mechanism must not be read as a
     * wrong direction and get flipped into driving the genuinely wrong way.
     *
     * Checked once per power-up, not once per move, so a move that starts inside
     * SIGN_CHECK_DEG of its target does not get a verdict from noise. If the
     * check ever did call it wrong, the move timeout still stops the servo.
     */
    private void checkSign(double error) {
        if (signLearned) {
            return;
        }
        // The answer has to be unambiguous, so only judge on a move big enough
        // that a real response dwarfs the threshold.
        if (Math.abs(error) < SIGN_CHECK_DEG * 3) {
            return;
        }
        if (signTimer.seconds() < SIGN_CHECK_SEC) {
            return;
        }

        double before = Math.abs(signCheckError);
        double now = Math.abs(error);

        if (now > before + SIGN_CHECK_DEG) {
            effectiveSign = -effectiveSign;
            signLearned = true;
        } else if (now < before - SIGN_CHECK_DEG) {
            signLearned = true;
        }

        signCheckError = error;
        signTimer.reset();
    }

    /**
     * Drive toward the zero stop until the angle stops responding, then back off
     * and call that zero. Recovers a reference with nothing remembered, which is
     * the contingency for auto not having run.
     */
    private void updateHoming() {
        if (moveTimer.seconds() > HOMING_TIMEOUT_SEC) {
            servo.setPower(0);
            state = State.FAILED;
            failure = String.format(
                    "deflector homing timed out after %.0fs. Never reached a stop -- "
                            + "check HOMING_DIRECTION and that the wire reads.",
                    HOMING_TIMEOUT_SEC);
            return;
        }

        if (homingBackingOff) {
            // Zero is already set; creep off the stop until clear of it.
            if (totalDegrees >= HOMING_BACKOFF_DEG) {
                servo.setPower(0);
                state = State.READY;
                mode = ControlMode.JOG;
                rawPower = 0;
                return;
            }
            servo.setPower(effectiveSign * -HOMING_DIRECTION * HOMING_POWER);
            return;
        }

        // Still hunting: has it stopped moving while being pushed?
        if (Math.abs(totalDegrees - stallReferenceAngle) > HOMING_STALL_DEG) {
            stallReferenceAngle = totalDegrees;
            stallTimer.reset();
        } else if (stallTimer.seconds() > HOMING_STALL_SEC) {
            // Against the stop. This is the reference.
            totalDegrees = 0;
            stallReferenceAngle = 0;
            homingBackingOff = true;
            return;
        }

        servo.setPower(effectiveSign * HOMING_DIRECTION * HOMING_POWER);
    }

    /** Begin homing. Finishes inside update() over the next several seconds. */
    public void startHoming() {
        state = State.HOMING;
        mode = ControlMode.HOLD;
        homingBackingOff = false;
        failure = null;
        stallReferenceAngle = totalDegrees;
        stallTimer.reset();
        moveTimer.reset();
    }

    /**
     * Declare the current physical position to be zero. This is what Prematch
     * uses once the driver has the deflector parked where they want it, and what
     * auto uses at INIT on the assumption that Prematch left it parked.
     */
    public void zeroHere() {
        totalDegrees = 0;
        targetAngle = 0;
        state = State.READY;
        failure = null;
    }

    /** Closed-loop target, in degrees from zero. Switches to HOLD. */
    public void setTargetAngle(double degrees) {
        targetAngle = degrees;
        mode = ControlMode.HOLD;
        failure = null;
        moveTimer.reset();
        // Baseline for the direction check, so it judges this move rather than
        // whatever the error happened to be before.
        signCheckError = targetAngle - totalDegrees;
        signTimer.reset();
    }

    /** The sign in use: +1 or -1. Differs from POWER_SIGN once a check flipped it. */
    public double getEffectiveSign() {
        return effectiveSign;
    }

    /** True once a commanded move has confirmed or corrected the direction. */
    public boolean isSignLearned() {
        return signLearned;
    }

    /** Raw power for the triggers, -1..1. Clamped to MAX_POWER. Switches to JOG. */
    public void setRawPower(double power) {
        rawPower = power;
        mode = ControlMode.JOG;
    }

    public void setControlMode(ControlMode newMode) {
        mode = newMode;
    }

    /** Cut power and stay put. */
    public void stop() {
        rawPower = 0;
        if (servo != null) {
            servo.setPower(0);
        }
    }

    /** Degrees from zero, counted through wraps. Meaningless unless READY. */
    public double getAngle() {
        return totalDegrees;
    }

    /** Where the wire says it is within one turn, 0-360. Always meaningful. */
    public double getRawAngle() {
        return lastRawAngle;
    }

    public double getVoltage() {
        return feedback == null ? 0 : feedback.getVoltage();
    }

    public double getMaxVoltage() {
        return feedback == null ? 0 : feedback.getMaxVoltage();
    }

    public State getState() {
        return state;
    }

    public ControlMode getControlMode() {
        return mode;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /** True once a move has landed inside tolerance. */
    public boolean atTarget() {
        return Math.abs(targetAngle - totalDegrees) <= TOLERANCE_DEG;
    }

    /** Why the last move or homing gave up, or null. */
    public String getFailure() {
        return failure;
    }

    public boolean isReady() {
        return state == State.READY;
    }

    private void accumulate() {
        double raw = readRawAngle();
        double change = raw - lastRawAngle;
        // A jump of more than half a turn in one loop is a wrap, not real motion.
        // Safe here because the servo cannot physically cover 180 deg in a loop.
        if (change > 180) change -= 360;
        if (change < -180) change += 360;
        totalDegrees += change;
        lastRawAngle = raw;
    }

    private double readRawAngle() {
        double max = feedback.getMaxVoltage();
        if (max <= 0) return 0;
        return feedback.getVoltage() / max * ASSUMED_SPAN_DEG;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
