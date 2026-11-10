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
    // until it stalls on the hard stop, call that -LEVEL_ABOVE_STOP_DEG, drive to
    // STOW_ANGLE_DEG, then CUT POWER -- G304.H: motionless once INIT is done.
    //
    // THE TWO ROBOTS NOW MEASURE ANGLES FROM DIFFERENT PLACES (2026-11-10):
    //
    //   6494  LEVEL_ABOVE_STOP_DEG = 0, so ZERO IS THE HARD STOP. Angles are
    //         degrees above the stop. They are NOT arc angles, because the stop
    //         is not level.
    //   5618  still NaN, so homing refuses and someone zeroes it by hand at
    //         level. Zero is level and angles ARE arc angles.
    //
    // So the SAME number means a different physical arc on each robot, and
    // anything shared between them -- DeflectorLogic.POSITIONS,
    // RouteAuto.AUTO_ARC_DEG -- cannot be right for both. Measure per robot
    // until one reference wins. To put 6494 back on arc angles, measure the
    // offset instead: Deflector Zero, jog LEVEL, A, then jog DOWN onto the stop
    // and read "Angle" -- e.g. -4.3 means 4.3.
    public static double LEVEL_ABOVE_STOP_DEG = Double.NaN;
    /**
     * Where homing PARKS, in degrees above the stop. This is the pre-match stow
     * position: the robot has to fit inside the 18 in. starting cube, so the
     * deflector cannot sit wherever it likes at the start of a match.
     *
     * 45 deg above the stop measures about 18 in. on 6494 (Jade, 2026-11-10), so
     * that is the pre-match park. 0 would mean parking on the stop itself.
     */
    public static double STOW_ANGLE_DEG = 45.0;

    public static double HOME_POWER = 0.25;          // down toward the stop; slow on purpose
    public static double HOME_SETTLE_SEC = 0.4;      // ignore stall checks while it starts moving
    public static double STALL_WINDOW_SEC = 0.25;    // check movement over this long...
    public static double STALL_SERVO_DEG = 3.0;      // ...less than this = sitting on the stop (~0.1 deflector deg)
    public static double HOME_DOWN_MAX_SEC = 12.0;   // no stop found by then = give up
    // Stop -> STOW_ANGLE_DEG. Was 5.0, which was sized for parking at level (0),
    // i.e. no travel at all. Parking at 45 deg is 1344 servo deg, about 3.7
    // TURNS, roughly 3.7 s at full speed plus the slow final approach -- so 5.0
    // was close enough to the real time to risk timing out. A timeout cuts power
    // wherever it got to, which at the start of a match could leave the robot
    // outside the 18 in. cube, so this errs long: INIT can be held as long as
    // anyone likes, and waiting costs nothing.
    public static double HOME_UP_MAX_SEC = 8.0;

    public enum HomeState { NOT_STARTED, DOWN, UP, DONE, FAILED }

    private static void loadRobotNumbers() {
        try {
            switch (WhichRobot.get()) {
                case R5618: LEVEL_ABOVE_STOP_DEG = Double.NaN; break;   // TODO measure (Deflector Zero)
                // 0 = the hard stop IS zero (Jade, 2026-11-10). Chosen over measuring
                // the offset to level: the stop is a physical fact, level is a
                // judgement. Cost: every angle is now measured from the stop, so it
                // is NOT an arc angle any more. See the class comment.
                case R6494: LEVEL_ABOVE_STOP_DEG = 0.0; break;
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
        target = STOW_ANGLE_DEG;   // park stowed unless someone asks for another angle meanwhile
        index = 0;
        homeTimer.reset();
        stallTimer.reset();
        stallRefTotal = total;
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
                boolean stalled = homeTimer.seconds() >= HOME_SETTLE_SEC
                        && Math.abs(total - stallRefTotal) < STALL_SERVO_DEG;
                stallRefTotal = total;
                stallTimer.reset();
                if (stalled) {
                    // On the stop, which is LEVEL_ABOVE_STOP_DEG below level.
                    zeroTotal = total + LEVEL_ABOVE_STOP_DEG * DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG;
                    zeroed = true;
                    keptZeroed = true;
                    keptZeroTotal = zeroTotal;
                    homeState = HomeState.UP;
                    homeTimer.reset();
                    homeNote = "homing: on the stop, up to level";
                    return 0;
                }
            }
            return -HOME_POWER * DeflectorLogic.POWER_SIGN;
        }
        // UP: to the stow angle, then power off. On 6494 that is a real traverse
        // of about 3.7 servo turns, not an instant finish -- see HOME_UP_MAX_SEC.
        double a = getAngle();
        boolean parked = Math.abs(a - STOW_ANGLE_DEG) <= DeflectorLogic.TOLERANCE_DEG;
        if (parked || homeTimer.seconds() > HOME_UP_MAX_SEC) {
            homeState = HomeState.DONE;
            homeNote = parked
                    ? String.format("HOMED, stowed at %.1f deg", STOW_ANGLE_DEG)
                    : String.format("homed, stopped at %.1f deg (wanted %.1f)", a, STOW_ANGLE_DEG);
            return 0;
        }
        return DeflectorLogic.power(a, STOW_ANGLE_DEG);
    }

    public boolean isHoming() { return homeState == HomeState.DOWN || homeState == HomeState.UP; }
    public HomeState getHomeState() { return homeState; }
    public String getHomeNote() { return homeNote; }

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
