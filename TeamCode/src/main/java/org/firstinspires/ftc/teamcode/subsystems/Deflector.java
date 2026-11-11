package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.WhichRobot;
import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;

/**
 * Deflector: sets the shot arc. 0 deg = level, 90 = straight up, positions in
 * DeflectorLogic.POSITIONS (0 / 67.5 / 70 / 72.5 / 75 / 80 / 90). The math and the "why" are
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

    // ---- Auto-homing (Shane, 2026-11-08) --------------------------------------
    // Both drive teams forgot Deflector Zero, so nothing shot. Now every auto
    // homes in INIT (and TeleOp at Start, if it isn't zeroed): jog DOWN slowly
    // until it stalls on the hard stop, call that -LEVEL_ABOVE_STOP_DEG, climb
    // to STOW_ANGLE_DEG, then CUT POWER -- G304.H: motionless once INIT is done.
    //
    // LEVEL_ABOVE_STOP_DEG is PER ROBOT (loadRobotNumbers). Measure it: Deflector
    // Zero, jog LEVEL, A, then jog DOWN onto the stop and read "Angle" -- e.g.
    // -4.3 means 4.3. NaN = not measured: homing refuses and you zero by hand.
    public static double LEVEL_ABOVE_STOP_DEG = Double.NaN;
    /**
     * Where homing parks, in degrees above ZERO. The pre-match stow: close to the
     * shooting angles so the first move in a match is short, while still inside
     * the 18 in. starting cube. 35 on 6494 (Shane, 2026-11-10).
     */
    public static double STOW_ANGLE_DEG = 35.0;

    /**
     * The arc auto, TeleOp and the tuner all go to at Start. ONE number on
     * purpose -- it was nearly kept in three places, which is how the deflector
     * angles got inconsistent before. 67 (Shane, 2026-11-10).
     */
    public static double DEFAULT_SHOT_DEG = 67.0;

    public static double HOME_POWER = 0.25;          // down toward the stop; slow on purpose
    public static double HOME_SETTLE_SEC = 0.4;      // ignore stall checks while it starts moving
    public static double STALL_WINDOW_SEC = 0.25;    // check movement over this long...
    public static double STALL_SERVO_DEG = 3.0;      // ...less than this = sitting on the stop (~0.1 deflector deg)

    /**
     * A stall only counts as HOME if the seek has travelled at least this far
     * first, in SERVO degrees. 60 is about a sixth of a turn.
     *
     * WITHOUT THIS, "not moving" was read as "on the stop" wherever it happened
     * to be sitting. On 2026-11-10 that set the zero at the stow, climbed 46.9
     * deg from there to about 82, and on the next run climbed another 46.9 to
     * about 129 -- the deflector ended up pointing the wrong way entirely. If it
     * has not moved, it is blocked or not being driven; that is a failure, not a
     * reference.
     */
    public static double HOME_MIN_TRAVEL_SERVO_DEG = 60.0;
    public static double HOME_DOWN_MAX_SEC = 12.0;   // no stop found by then = give up
    // ---- The climb from HOME up to the stow ----------------------------------
    // Done on JOG POWER, not DeflectorLogic.power(). The position loop commands
    // kP * error floored at MIN_POWER = 0.05, and 5% will not lift this deflector
    // through its ~30:1 -- it stalls short and in a different place every run.
    // Proved in Deflector Auto Home (Shane, 2026-11-10) before being moved here.
    public static double HOME_RISE_POWER = 0.25;      // fast part
    public static double HOME_RISE_SLOW_DEG = 2.0;    // ease off inside this
    public static double HOME_RISE_SLOW_POWER = 0.12; // still enough to lift it

    // HOME to a 35 deg stow is 46.9 deg, about FOUR servo turns. The old 5.0 was
    // sized for climbing to level (one turn) and would time out mid-climb.
    public static double HOME_UP_MAX_SEC = 20.0;

    public enum HomeState { NOT_STARTED, DOWN, UP, DONE, FAILED }

    private static void loadRobotNumbers() {
        try {
            switch (WhichRobot.get()) {
                case R5618: LEVEL_ABOVE_STOP_DEG = Double.NaN; break;   // TODO measure (Deflector Zero)
                // 11.9 measured and confirmed repeatable with Deflector Auto Home
                // (Shane, 2026-11-10). This is what switches auto-homing ON.
                case R6494: LEVEL_ABOVE_STOP_DEG = 11.9; break;
            }
        } catch (IllegalStateException e) {
            LEVEL_ABOVE_STOP_DEG = Double.NaN;   // unknown robot: don't guess
        }
    }

    private HomeState homeState = HomeState.NOT_STARTED;
    private String homeNote = "not homed";
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallRefTotal = 0;
    private double seekStartTotal = 0;

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
    private boolean jogIgnoresLimits = false;
    private double maxStepSeen = 0;   // biggest servo-degree change in one loop

    public Deflector(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(CRServo.class, SERVO_NAME);
        feedback = hardwareMap.tryGet(AnalogInput.class, FEEDBACK_NAME);
        if (!isPresent()) return;
        loadRobotNumbers();

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

        // Homing was asked for explicitly, so it moves even in INIT.
        if (isHoming()) {
            appliedPower = homingPower();
            servo.setPower(appliedPower);
            return;
        }

        if (!allowMotion) {
            appliedPower = 0;
        } else if (jogging) {
            appliedPower = jogPower * DeflectorLogic.POWER_SIGN;
            // Limits mean something only once zeroed -- and not while someone is
            // re-finding level (Deflector Zero): an old, wrong zero would wall it off.
            if (zeroed && !jogIgnoresLimits) {
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

    /**
     * true = jog() ignores the soft limits even when zeroed. ONLY for Deflector
     * Zero, where a person is watching and the old zero may be the thing that's
     * wrong. Hold-at-target (D-pad steps) still respects the limits.
     */
    public void setJogIgnoresLimits(boolean ignore) { jogIgnoresLimits = ignore; }

    /**
     * Start auto-homing (see LEVEL_ABOVE_STOP_DEG). Runs inside update(), even
     * with allowMotion false. Ends at level with the power off. If this robot's
     * number isn't measured it refuses (FAILED) and any old zero is kept.
     */
    public void startHoming() {
        if (!isPresent()) return;
        if (Double.isNaN(LEVEL_ABOVE_STOP_DEG)) {
            homeState = HomeState.FAILED;
            homeNote = "auto-home OFF: stop angle not measured for this robot -- run Deflector Zero";
            return;
        }
        homeState = HomeState.DOWN;
        jogging = false;
        target = STOW_ANGLE_DEG;   // park stowed unless something asks otherwise mid-run
        index = 0;
        homeTimer.reset();
        stallTimer.reset();
        stallRefTotal = total;
        seekStartTotal = total;
        homeNote = "homing: down to the stop";
    }

    private double homingPower() {
        if (homeState == HomeState.DOWN) {
            if (homeTimer.seconds() > HOME_DOWN_MAX_SEC) {
                homeState = HomeState.FAILED;
                homeNote = String.format("HOMING FAILED: no stop found in %.0f s -- run Deflector Zero",
                        HOME_DOWN_MAX_SEC);
                return 0;
            }
            if (stallTimer.seconds() >= STALL_WINDOW_SEC) {
                double travelled = Math.abs(total - seekStartTotal);
                boolean notMoving = homeTimer.seconds() >= HOME_SETTLE_SEC
                        && Math.abs(total - stallRefTotal) < STALL_SERVO_DEG;
                stallRefTotal = total;
                stallTimer.reset();

                // Not moving AND never moved = blocked or not being driven. Do
                // NOT call that the stop: setting the zero there is what pointed
                // the deflector the wrong way twice on 2026-11-10.
                if (notMoving && travelled < HOME_MIN_TRAVEL_SERVO_DEG) {
                    homeState = HomeState.FAILED;
                    homeNote = String.format(
                            "HOMING FAILED: never moved (%.0f of %.0f servo deg). Blocked, or "
                                    + "the servo is not driving. Zero NOT set.",
                            travelled, HOME_MIN_TRAVEL_SERVO_DEG);
                    return 0;
                }

                if (notMoving) {
                    // On the stop, which is LEVEL_ABOVE_STOP_DEG below level.
                    zeroTotal = total + LEVEL_ABOVE_STOP_DEG * DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG;
                    zeroed = true;
                    keptZeroed = true;
                    keptZeroTotal = zeroTotal;
                    homeState = HomeState.UP;
                    homeTimer.reset();
                    homeNote = String.format("homing: on the stop, climbing to the %.0f stow", STOW_ANGLE_DEG);
                    return 0;
                }
            }
            return -HOME_POWER * DeflectorLogic.POWER_SIGN;
        }
        // UP: climb to the stow on jog power, then cut power. See the constants.
        double a = getAngle();
        double remaining = STOW_ANGLE_DEG - a;

        if (remaining <= 0) {
            homeState = HomeState.DONE;
            homeNote = String.format("HOMED, stowed at %.1f deg", a);
            // DO NOT touch target here. startHoming() already set it to the stow, so
            // the hold loop keeps it there -- and if something called goTo()
            // mid-homing (pressing Start before homing finished, which RouteAuto
            // and DriveTeleOp both allow), that target has to survive. Setting it
            // to the current angle here threw the requested shot angle away and
            // parked at the stow instead.
            return 0;
        }
        if (homeTimer.seconds() > HOME_UP_MAX_SEC) {
            homeState = HomeState.FAILED;
            homeNote = String.format(
                    "HOMING FAILED: stopped %.1f deg short of the %.1f stow -- "
                            + "HOME_RISE_SLOW_POWER may be too weak", remaining, STOW_ANGLE_DEG);
            return 0;
        }
        return DeflectorLogic.POWER_SIGN * (remaining > HOME_RISE_SLOW_DEG
                ? HOME_RISE_POWER : HOME_RISE_SLOW_POWER);
    }

    public boolean isHoming() { return homeState == HomeState.DOWN || homeState == HomeState.UP; }
    public HomeState getHomeState() { return homeState; }
    public String getHomeNote() { return homeNote; }

    /**
     * "The deflector is at THIS angle right now" -- set the zero by arithmetic
     * instead of by driving somewhere.
     *
     * zeroAt(-11.9) while sitting on the hard stop means HOME is 11.9 below
     * level, so level becomes 0 with nothing having to move. Same offset the
     * homing path computes internally; this exposes it so an op mode can set the
     * reference from a place it can actually reach.
     *
     * Driving to level and calling zeroHere() there looks equivalent and is not:
     * it makes the reference depend on the position loop closing its last degree
     * uphill, which it cannot. DeflectorLogic.MIN_POWER runs out of authority
     * around 0.7 deg of error, so an upward move stalls short of tolerance and
     * never reports arriving.
     */
    public void zeroAt(double angleHere) {
        if (!isPresent()) return;
        homeState = HomeState.NOT_STARTED;
        homeNote = "zeroed by hand";
        zeroTotal = total - angleHere * DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG;
        zeroed = true;
        keptZeroed = true;
        keptZeroTotal = zeroTotal;
        // HOLD STILL. Not clipToLimits(angleHere): angleHere is normally BELOW
        // MIN_ANGLE (the hard stop is outside the soft limits by definition), so
        // clipping turned -11.9 into -2.0 and the hold loop immediately started
        // climbing 9.9 deg at MIN_POWER and stalling at random. Setting the
        // reference must not command a move.
        target = angleHere;
        index = DeflectorLogic.nearestIndex(target);
    }


    /** "It's level right now": this is 0 degrees. Kept for later op modes. */
    public void zeroHere() {
        if (!isPresent()) return;
        homeState = HomeState.NOT_STARTED;   // a hand zero replaces any homing in progress
        homeNote = "zeroed by hand";
        zeroTotal = total;
        zeroed = true;
        keptZeroed = true;
        keptZeroTotal = zeroTotal;
        target = 0;
        index = 0;
    }

    /** Forget the zero (it will refuse to move until zeroed again). */
    public void clearZero() {
        homeState = HomeState.NOT_STARTED;
        homeNote = "zero cleared";
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
