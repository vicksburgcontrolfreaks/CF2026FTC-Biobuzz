package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;

/**
 * Deflector: sets the shot arc. 0 deg = level, 90 = straight up, positions in
 * DeflectorLogic.POSITIONS (0 / 60 / 65 / 70 / 75 / 80 / 90). The math and the "why" are
 * in DeflectorLogic.
 *
 * Config (6494): CR servo "deflector", its feedback "deflectorPos" (Analog Input,
 * Control Hub analog 1). Either missing -> isPresent() false and everything here
 * does nothing, so op modes still run on a robot without one.
 *
 * ZEROING -- EVERY MATCH. The feedback can't tell which of its ~7.5 turns it's
 * on, so at power-up the angle is unknown and the deflector WILL NOT MOVE until
 * someone zeroes it:
 *
 *     Deflector deflector = new Deflector(hardwareMap);
 *     // loop:
 *     deflector.update(false);          // track it, don't drive it
 *     if (button) deflector.zeroHere(); // while it's held LEVEL
 *
 * That's all a pre-match op mode needs. The zero is then KEPT (in statics) for
 * every op mode after it -- auto, TeleOp -- until a redeploy or power cycle,
 * same idea as the turret's kept homing. Each op mode picks up the turn count
 * where the last one left off, which assumes nobody turned the deflector more
 * than ~6 degrees by hand in between.
 *
 * Usage in a match op mode:
 *     deflector.stepUp() / stepDown() / goToIndex(i)
 *     deflector.update(true);           // every loop, after commands
 */
public class Deflector {

    public static final String SERVO_NAME = "deflector";
    public static final String FEEDBACK_NAME = "deflectorPos";

    // Kept across op modes (statics survive until a redeploy or power cycle).
    private static boolean keptZeroed = false;
    private static double keptZeroTotal = 0;
    private static double keptTotal = 0;
    private static double keptLastRaw = Double.NaN;

    private final CRServo servo;
    private final AnalogInput feedback;

    private double total;        // running servo degrees
    private double lastRaw;
    private double zeroTotal;
    private boolean zeroed;
    private double target;       // deflector degrees
    private int index;
    private double appliedPower = 0;
    private boolean jogging = false;
    private double jogPower = 0;
    private double maxStepSeen = 0;   // biggest servo-degree change in one loop

    public Deflector(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(CRServo.class, SERVO_NAME);
        feedback = hardwareMap.tryGet(AnalogInput.class, FEEDBACK_NAME);
        if (!isPresent()) return;

        servo.setPower(0);
        double raw = readRaw();
        if (keptZeroed && !Double.isNaN(keptLastRaw)) {
            // Carry on from the last op mode's count.
            total = keptTotal + DeflectorLogic.unwrapStep(keptLastRaw, raw);
            zeroTotal = keptZeroTotal;
            zeroed = true;
        } else {
            total = 0;
            zeroed = false;
        }
        lastRaw = raw;
        target = zeroed ? getAngle() : 0;   // hold where it is; don't lurch
        index = DeflectorLogic.nearestIndex(target);
        remember();
    }

    public boolean isPresent() {
        return servo != null && feedback != null;
    }

    /**
     * Every loop. Tracks the turn count always; drives the servo only when
     * allowMotion is true AND it has been zeroed. Pass false during INIT.
     */
    public void update(boolean allowMotion) {
        if (!isPresent()) return;
        double raw = readRaw();
        double step = DeflectorLogic.unwrapStep(lastRaw, raw);
        // Near 180 = the unwrap is about to guess wrong (see DeflectorLogic).
        maxStepSeen = Math.max(maxStepSeen, Math.abs(step));
        total += step;
        lastRaw = raw;
        remember();

        if (!allowMotion) {
            appliedPower = 0;
        } else if (jogging) {
            appliedPower = jogPower * DeflectorLogic.POWER_SIGN;
            if (zeroed) {   // limits mean something only once zeroed
                double a = getAngle();
                if (a >= DeflectorLogic.MAX_ANGLE && jogPower > 0) appliedPower = 0;
                if (a <= DeflectorLogic.MIN_ANGLE && jogPower < 0) appliedPower = 0;
            }
        } else {
            appliedPower = zeroed ? DeflectorLogic.power(getAngle(), target) : 0;
        }
        servo.setPower(appliedPower);
    }

    /**
     * Drive by hand, every loop: + = up. Works BEFORE zeroing (that's how you
     * get it level). Pass 0 to let go: it then holds where it stopped (if zeroed).
     * Soft limits apply once zeroed.
     */
    public void jog(double power) {
        boolean nowJogging = power != 0;
        if (jogging && !nowJogging && zeroed) {
            target = DeflectorLogic.clipToLimits(getAngle());   // hold here, don't snap back
            index = DeflectorLogic.nearestIndex(target);
        }
        jogging = nowJogging;
        jogPower = Math.max(-1, Math.min(1, power));
    }

    /** "It's level right now": this is 0 degrees. Kept for later op modes. */
    public void zeroHere() {
        if (!isPresent()) return;
        zeroTotal = total;
        zeroed = true;
        keptZeroed = true;
        keptZeroTotal = zeroTotal;
        target = 0;
        index = 0;
    }

    /** Forget the zero (it will refuse to move until zeroed again). */
    public void clearZero() {
        zeroed = false;
        keptZeroed = false;
    }

    /** Go to an angle (deflector degrees), clipped to the soft limits. */
    public void goTo(double degrees) {
        target = DeflectorLogic.clipToLimits(degrees);
        index = DeflectorLogic.nearestIndex(target);
    }

    public void goToIndex(int i) {
        index = Math.max(0, Math.min(DeflectorLogic.POSITIONS.length - 1, i));
        target = DeflectorLogic.clipToLimits(DeflectorLogic.POSITIONS[index]);
    }

    /** One position higher (bigger arc angle). Stays put at the top. */
    public void stepUp()   { goToIndex(index + 1); }
    /** One position lower. Stays put at the bottom. */
    public void stepDown() { goToIndex(index - 1); }

    public void stop() {
        if (servo != null) servo.setPower(0);
        appliedPower = 0;
    }

    private double readRaw() {
        return feedback.getVoltage() / feedback.getMaxVoltage() * 360.0;
    }

    private void remember() {
        keptTotal = total;
        keptLastRaw = lastRaw;
    }

    // ---- Readouts -----------------------------------------------------------

    public boolean isZeroed()   { return zeroed; }
    /** Deflector degrees (meaningless until zeroed). */
    public double getAngle()    { return DeflectorLogic.toDeflectorDeg(total, zeroTotal); }
    public double getTarget()   { return target; }
    public int getIndex()       { return index; }
    public boolean isOnTarget() { return zeroed && Math.abs(target - getAngle()) <= DeflectorLogic.TOLERANCE_DEG; }
    public double getPower()    { return appliedPower; }
    /** Running servo degrees -- for debugging the unwrap. */
    public double getServoTotal() { return total; }
    /** Biggest servo-degree move between two updates since this op mode started. Near 180 = turns can be miscounted. */
    public double getMaxStepSeen() { return maxStepSeen; }
}
