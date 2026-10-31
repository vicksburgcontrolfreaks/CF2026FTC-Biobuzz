package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter pivot: the servo that sets the gap between the two flywheels, so the
 * shooter can hold a pollen ball or a (larger) nectar ball.
 *
 * Driver Station config names this expects:
 *   "shooterPivot" - Servo, on a servo port. REQUIRED.
 *   "pivotHome"    - the IR beam break, on a digital port. OPTIONAL -- see
 *                    below. There is no such sensor on the robot right now.
 *
 * HOW IT KNOWS WHERE IT IS: IT JUST DOES
 * setPosition(0.539) is absolute. A servo has a potentiometer on its output
 * shaft and its own control loop, so that command means one shaft angle and it
 * means the same angle every power cycle, forever. Unlike a motor -- which
 * counts encoder ticks from wherever it happened to be sitting at INIT -- there
 * is nothing to recover at startup. The measured positions below are all this
 * class needs to work, and the normal path through it never homes.
 *
 * WHY THERE IS HOMING CODE HERE ANYWAY
 * Not to find the servo. To find the MECHANISM. What is not stable is the
 * MAPPING from servo position to physical flywheel gap: re-mount the servo, pop
 * the linkage off, let a horn slip one spline, and 0.539 is no longer the pollen
 * gap -- but the code still says 0.539 and the shots just quietly go bad.
 *
 * home() sweeps the servo until an IR beam trips, and calls the position that
 * did it zero. Pollen and nectar then live as offsets FROM zero rather than as
 * absolute positions, so re-homing after a repair lands them in the right
 * physical place with nobody re-measuring anything.
 *
 * WHY IT IS DORMANT, AND THE DATA THAT SAYS LEAVE IT THAT WAY  (2026-10-31)
 * The mechanism really did move between 2026-10-01 and 2026-10-30, which is
 * exactly the case homing exists for. But look at how much it moved:
 *
 *     pollen  0.539 - 0.48  = +0.059
 *     nectar  0.21  - 0.135 = +0.075
 *
 * Those differ by 0.016, so the shift was NOT a constant offset. A single
 * reference point -- which is all a homing sensor ever finds -- can only
 * correct a constant offset, so re-homing would have left about 0.016 of error
 * on one of the two grips. Nectar sits 0.044 from the widest the mechanism
 * goes, so 0.016 is a third of its entire clearance. Not nothing.
 *
 * Measuring each grip against its real ball with Pivot Jog Test tests the thing
 * that actually matters -- does the ball fit -- instead of testing a proxy and
 * then trusting an assumption the data does not support. That is a few minutes
 * of work after a repair, and it is the recommended procedure.
 *
 * So: no beam break is fitted, none is needed, and this code is kept rather
 * than deleted because it costs nothing to keep and the decision may change
 * (a mechanism that slips OFTEN would tip the other way, since auto-correcting
 * at every INIT beats remembering to re-measure). If you do fit one, wire it,
 * run ShooterPivotHomeTest, measure POLLEN_OFFSET and NECTAR_OFFSET, and this
 * path comes alive with no other changes.
 *
 * TWO THINGS THAT WILL BITE YOU IF YOU EVER DO HOME, and what this class
 * already does about them:
 *
 *   1. SERVO LAG. A servo does not teleport. Command 0.30, read the beam a
 *      millisecond later, and you are reading the beam at 0.28 because the servo
 *      is still on its way. Record that and your zero is wrong -- wrong by a
 *      different amount every run, depending on load. Fixed by dwelling after
 *      each step so the servo actually arrives. If the homed zero is not
 *      repeatable run to run, DWELL IS THE FIRST THING TO INCREASE.
 *
 *   2. LINKAGE SLOP. Gears and linkages have backlash, so the beam trips at a
 *      slightly different place coming from below than from above. Fixed by
 *      always making the final approach in the same direction: creep up on the
 *      beam, back off, then creep up on it again the same way.
 *
 * Homing is optional in both senses: construct without a beam break and home()
 * simply reports why it cannot run, and even with one fitted the pivot works
 * off the measured absolute positions until home() succeeds AND the offsets
 * have been measured.
 */
@Configurable
public class ShooterPivot {

    // ---- Beam break wiring --------------------------------------------------
    // REV beam breaks pull the line LOW when the beam is blocked, which is why
    // the old magazine code read !sensor1.getState(). If "Beam Broken" in
    // ShooterPivotHomeTest reads backwards from reality, flip this.
    public static boolean BEAM_BROKEN_STATE = false;

    // ---- Sweep shape --------------------------------------------------------
    // The sweep runs FROM -> TO, so swapping these two reverses the direction.
    // If the beam is already blocked at SWEEP_FROM, that is the fix: the flag is
    // sitting on the near side, so start from the other end instead.
    public static double SWEEP_FROM = 0.0;
    public static double SWEEP_TO = 1.0;

    // Phase 1 covers ground fast. Phase 2 creeps, for an accurate edge.
    public static double COARSE_STEP = 0.01;
    public static long COARSE_DWELL_MS = 15;
    public static double BACKOFF = 0.03;   // must be comfortably bigger than COARSE_STEP
    public static double FINE_STEP = 0.002;
    public static long FINE_DWELL_MS = 25;

    public static double HOME_TIMEOUT_SEC = 6.0;

    // ---- Where the gaps are, relative to the homed zero ---------------------
    // UNMEASURED, and only used if a beam break is ever fitted and home()
    // succeeds. To measure them: wire the sensor, run ShooterPivotHomeTest, home
    // the pivot, jog until a pollen ball just fits, and read "Offset From Zero"
    // off the telemetry. Repeat for nectar.
    //
    // NaN, NOT 0.0, and that is deliberate. 0.0 is a perfectly legal offset, so
    // a 0.0 here is indistinguishable from "nobody has measured this yet" -- and
    // the failure it caused would be silent: the day somebody powers the sensor,
    // homing starts succeeding, both grips go to zero + 0.0, pollen and nectar
    // end up at the SAME gap, and the measured numbers below are still sitting
    // right there in the file but no longer reachable. Nobody touched the pivot
    // code, so nobody suspects it. NaN cannot be mistaken for a measurement, so
    // goToPollen/goToNectar keep using the measured positions until these are
    // filled in. Same trick as OPEN_POSITION below.
    public static double POLLEN_OFFSET = Double.NaN;
    public static double NECTAR_OFFSET = Double.NaN;

    // ---- Safe travel band --------------------------------------------------
    // MEASURED ON THE ROBOT 2026-10-30 with Pivot Jog Test: jogged until the
    // servo audibly strained at each end, then backed off. Everything that
    // commands the pivot is clipped to this band, which is what makes it
    // impossible to drive into a hard stop -- the failure that hunted this servo
    // twice on 2026-10-30, when the clip was still the SDK range of 0.0-1.0.
    //
    // CONFIRMED 2026-10-31: both ends are the MECHANISM's hard stops, not the
    // servo's own travel limits -- which is also why they sit well inside the
    // SDK's 0.0-1.0. That matters for two reasons. The mechanism has physical
    // references at both ends, so Pivot Jog Test can always find them again,
    // which is what makes re-measuring a reliable substitute for a homing
    // sensor. And these two numbers move WITH the linkage: re-seat it and the
    // band shifts along with the grips, so re-measure all four together.
    //
    // LOWER IS A WIDER GAP. Nectar is the bigger ball and sits at the low end.
    public static double MIN_SAFE_POSITION = 0.166;   // widest
    public static double MAX_SAFE_POSITION = 0.679;   // tightest

    // The wide-open position, for pulling a stuck ball out without anyone
    // forcing the servo by hand. MEASURED ON THE ROBOT 2026-10-30 with Pivot
    // Jog Test. goToOpen() returns false instead of guessing if this is ever
    // set back to NaN.
    public static double OPEN_POSITION = 0.2;

    // THE TWO NUMBERS THAT RUN THE SHOOTER. goToPollen()/goToNectar() use these
    // unless a beam break is fitted, home() succeeded, and the offsets above
    // have been measured -- none of which is true today, and none of which has
    // to become true. These were called FALLBACK_* until 2026-10-31, which made
    // the primary path sound like a backup plan.
    //
    // MEASURED ON THE ROBOT 2026-10-30 with Pivot Jog Test, against real balls.
    // RE-MEASURE THEM THE SAME WAY after any work on the servo, the horn or the
    // linkage -- that is this mechanism's calibration procedure, and it replaces
    // what a homing sensor would have done automatically.
    //
    // The 2026-10-01 bench values were pollen 0.48 and nectar 0.135. Both moved
    // up -- by +0.059 and +0.075, which is a re-seated linkage or horn rather
    // than anything changing about the balls. That the two shifts DIFFER is the
    // whole argument for re-measuring instead of homing; see the class comment.
    public static double POLLEN_POSITION = 0.539;
    public static double NECTAR_POSITION = 0.21;

    private final Servo pivot;
    private final DigitalChannel beam;

    private boolean homed = false;
    private double zeroPosition = 0.0;
    private double commandedPosition = 0.0;
    private String homeFailure = null;

    /**
     * The normal way to build one: just the servo, no homing sensor. This is
     * the whole robot as it stands -- the pivot runs off POLLEN_POSITION and
     * NECTAR_POSITION and never needs a digital port.
     */
    public ShooterPivot(Servo pivot) {
        this(pivot, null);
    }

    /**
     * With a homing sensor. beam may be null, which is the same as using the
     * one-argument constructor -- so an op mode can pass whatever
     * hardwareMap.tryGet() handed back without checking it first.
     */
    public ShooterPivot(Servo pivot, DigitalChannel beam) {
        this.pivot = pivot;
        this.beam = beam;
        if (beam != null) {
            beam.setMode(DigitalChannel.Mode.INPUT);
        }
    }

    /** True if a pivotHome beam break was passed in, so home() can run. */
    public boolean hasBeamBreak() {
        return beam != null;
    }

    /**
     * True when something is blocking the IR beam. Always false when no beam
     * break is fitted, so gate on hasBeamBreak() before believing it.
     */
    public boolean isBeamBroken() {
        return beam != null && isBeamBroken(beam);
    }

    /**
     * Polarity-aware beam read that does not need a ShooterPivot instance, so a
     * bring-up op mode can check the sensor before the pivot servo is wired.
     */
    public static boolean isBeamBroken(DigitalChannel beam) {
        return beam.getState() == BEAM_BROKEN_STATE;
    }

    public boolean isHomed() {
        return homed;
    }

    /** The servo position that tripped the beam. Meaningful only once homed. */
    public double getZeroPosition() {
        return zeroPosition;
    }

    public double getCommandedPosition() {
        return commandedPosition;
    }

    /** How far the pivot sits from zero right now -- the number to write down. */
    public double getOffsetFromZero() {
        return commandedPosition - zeroPosition;
    }

    /** Why the last home() failed, or null if it succeeded or has not run. */
    public String getHomeFailure() {
        return homeFailure;
    }

    /**
     * Sweeps until the beam trips and stores that servo position as zero.
     * Blocking, and takes roughly two seconds. Returns false if it could not
     * find the beam, in which case getHomeFailure() explains why.
     */
    public boolean home() {
        homed = false;
        homeFailure = null;

        if (beam == null) {
            homeFailure = "no pivotHome beam break fitted - the pivot is running "
                    + "on the positions measured with Pivot Jog Test, which is "
                    + "the normal case and not a problem";
            return false;
        }

        ElapsedTime timer = new ElapsedTime();

        double direction = Math.signum(SWEEP_TO - SWEEP_FROM);
        if (direction == 0) {
            homeFailure = "SWEEP_FROM and SWEEP_TO are the same value - nothing to sweep";
            return false;
        }

        // Get to the start of travel. This is the longest single move of the
        // whole routine, so it gets the longest dwell.
        moveTo(SWEEP_FROM, 400);

        // The sweep looks for a CLEAR -> BROKEN transition. Starting already
        // blocked means there is no such transition ahead of us.
        if (isBeamBroken()) {
            homeFailure = "beam already blocked at SWEEP_FROM (" + SWEEP_FROM
                    + ") - swap SWEEP_FROM and SWEEP_TO to sweep the other way";
            return false;
        }

        // Phase 1: coarse approach.
        Double coarseHit = sweepUntilBeamBreaks(SWEEP_FROM, SWEEP_TO, COARSE_STEP,
                COARSE_DWELL_MS, direction, timer);
        if (coarseHit == null) {
            homeFailure = (timer.seconds() >= HOME_TIMEOUT_SEC)
                    ? "timed out after " + HOME_TIMEOUT_SEC + "s during coarse sweep"
                    : "swept the whole range " + SWEEP_FROM + " -> " + SWEEP_TO
                      + " and never saw the beam break";
            return false;
        }

        // Phase 2: back off far enough to clear the beam, then creep back in so
        // the final approach direction matches every other run.
        double backoffTarget = clip(coarseHit - direction * BACKOFF);
        moveTo(backoffTarget, 150);
        if (isBeamBroken()) {
            // BACKOFF too small to escape the flag. The fine approach would then
            // start already-broken and record the backoff point as zero.
            homeFailure = "still blocked after backing off " + BACKOFF
                    + " - increase BACKOFF";
            return false;
        }

        Double fineHit = sweepUntilBeamBreaks(backoffTarget, SWEEP_TO, FINE_STEP,
                FINE_DWELL_MS, direction, timer);
        if (fineHit == null) {
            homeFailure = (timer.seconds() >= HOME_TIMEOUT_SEC)
                    ? "timed out after " + HOME_TIMEOUT_SEC + "s during fine sweep"
                    : "lost the beam on the fine approach - try a longer FINE_DWELL_MS";
            return false;
        }

        zeroPosition = fineHit;
        homed = true;
        return true;
    }

    /**
     * Steps from start toward limit until the beam breaks. Returns the position
     * that broke it, or null on timeout or on running out of travel.
     */
    private Double sweepUntilBeamBreaks(double start, double limit, double step,
                                        long dwellMs, double direction,
                                        ElapsedTime timer) {
        double position = start;
        while (timer.seconds() < HOME_TIMEOUT_SEC) {
            // direction * (limit - position) goes negative once we pass limit,
            // which is the same test for both sweep directions.
            if (direction * (limit - position) < 0) {
                return null;
            }
            moveTo(position, dwellMs);
            if (isBeamBroken()) {
                return position;
            }
            position += direction * step;
        }
        return null;
    }

    /**
     * Sets pollen spacing. Uses the measured absolute position unless homing
     * has actually happened AND POLLEN_OFFSET has actually been measured.
     */
    public void goToPollen() {
        if (isUsingHomedOffsets()) {
            setOffsetFromZero(POLLEN_OFFSET);
        } else {
            setPosition(POLLEN_POSITION);
        }
    }

    /**
     * Sets nectar spacing. Uses the measured absolute position unless homing
     * has actually happened AND NECTAR_OFFSET has actually been measured.
     */
    public void goToNectar() {
        if (isUsingHomedOffsets()) {
            setOffsetFromZero(NECTAR_OFFSET);
        } else {
            setPosition(NECTAR_POSITION);
        }
    }

    /**
     * True when the pivot is driving off a homed zero plus offsets rather than
     * off the measured absolute positions. Needs BOTH halves: a successful home
     * is useless without measured offsets, and measured offsets are meaningless
     * without a zero to measure them from. Telemetry should show this rather
     * than isHomed(), because isHomed() alone does not say which numbers are in
     * use.
     */
    public boolean isUsingHomedOffsets() {
        return homed && !Double.isNaN(POLLEN_OFFSET) && !Double.isNaN(NECTAR_OFFSET);
    }

    /** Moves to zero + offset. Only meaningful once homed. */
    public void setOffsetFromZero(double offset) {
        setPosition(zeroPosition + offset);
    }

    /**
     * Moves the pivot wide open, so a stuck ball can be pulled out without
     * anyone forcing the servo by hand. Back-driving a powered servo can slip
     * its pot or bind the linkage, and then it hunts for a position it can no
     * longer reach -- that happened on 2026-10-30.
     *
     * Returns false and does nothing if OPEN_POSITION has not been measured yet.
     */
    public boolean goToOpen() {
        if (Double.isNaN(OPEN_POSITION)) {
            return false;
        }
        setPosition(OPEN_POSITION);
        return true;
    }

    /** Raw absolute servo position, clipped to the safe travel band. */
    public void setPosition(double position) {
        commandedPosition = clip(position);
        pivot.setPosition(commandedPosition);
    }

    private void moveTo(double position, long dwellMs) {
        setPosition(position);
        sleep(dwellMs);
    }

    /**
     * Clips to the MECHANISM's travel, not the SDK's. 0.0 and 1.0 are merely
     * what a servo will accept -- past the mechanism's real limits the servo is
     * pressed against a hard stop, where it strains, heats and eventually
     * cooks. Measure the band with Pivot Jog Test and put it above.
     */
    private double clip(double position) {
        return Range.clip(position, MIN_SAFE_POSITION, MAX_SAFE_POSITION);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
