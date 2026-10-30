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
 *   "shooterPivot" - Servo, on a servo port
 *   "pivotHome"    - the IR beam break, on a digital port
 *
 * WHY A SERVO NEEDS "HOMING" AT ALL
 * Normally it does not. setPosition(0.48) is absolute -- the servo goes to the
 * same place every time, so there is nothing to recover. What is NOT stable is
 * the MAPPING from servo position to physical flywheel gap. Re-mount the servo,
 * pop the linkage off, let a horn slip one spline, and 0.48 is no longer the
 * pollen gap -- but the code still says 0.48 and the shots just quietly go bad.
 *
 * So this class does not measure where the servo is. It measures where the
 * MECHANISM is, by sweeping the servo until the IR beam trips and recording the
 * servo position that did it. That position becomes zero, and pollen/nectar are
 * stored as offsets FROM zero instead of as absolute positions. Re-home after a
 * repair and the offsets still land in the right physical place.
 *
 * TWO THINGS THAT WILL BITE YOU, and what this class does about them:
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
 * Call home() once before the pivot is used. Until then this class falls back to
 * the bench-measured absolute positions so the robot still shoots.
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
    // TODO(on-robot): BOTH OF THESE ARE PLACEHOLDERS. Run ShooterPivotHomeTest,
    // home the pivot, then jog the servo until a pollen ball just fits and read
    // the "Offset From Zero" number off the telemetry. Repeat for nectar. Put
    // those two numbers here. Until then the pivot homes fine and then sits at
    // zero, which is almost certainly not a useful flywheel gap.
    public static double POLLEN_OFFSET = 0.0;
    public static double NECTAR_OFFSET = 0.0;

    // ---- Safe travel band --------------------------------------------------
    // TODO(on-robot): PLACEHOLDERS, deliberately the full legal range so that
    // setting them changes nothing until they are measured. Find the real ends
    // with Pivot Jog Test -- jog until the servo audibly strains, back off, and
    // record. Everything that commands the pivot is clipped to this band, so
    // correct numbers here make it impossible to drive into a hard stop.
    public static double MIN_SAFE_POSITION = 0.0;
    public static double MAX_SAFE_POSITION = 1.0;

    // TODO(on-robot): the wide-open position, for pulling a stuck ball out.
    // NaN until measured, and goToOpen() refuses to move while it is NaN rather
    // than guessing and slamming the mechanism into something.
    public static double OPEN_POSITION = Double.NaN;

    // Used only if homing fails, so a failed home degrades to "the old numbers"
    // instead of "the shooter does not work". Bench-measured as ABSOLUTE servo
    // positions on 2026-10-01, before homing existed.
    public static double FALLBACK_POLLEN_POSITION = 0.48;
    public static double FALLBACK_NECTAR_POSITION = 0.135;

    private final Servo pivot;
    private final DigitalChannel beam;

    private boolean homed = false;
    private double zeroPosition = 0.0;
    private double commandedPosition = 0.0;
    private String homeFailure = null;

    public ShooterPivot(Servo pivot, DigitalChannel beam) {
        this.pivot = pivot;
        this.beam = beam;
        this.beam.setMode(DigitalChannel.Mode.INPUT);
    }

    /** True when something is blocking the IR beam. */
    public boolean isBeamBroken() {
        return isBeamBroken(beam);
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

    /** Sets pollen spacing, measured out from the homed zero. */
    public void goToPollen() {
        if (homed) {
            setOffsetFromZero(POLLEN_OFFSET);
        } else {
            setPosition(FALLBACK_POLLEN_POSITION);
        }
    }

    /** Sets nectar spacing, measured out from the homed zero. */
    public void goToNectar() {
        if (homed) {
            setOffsetFromZero(NECTAR_OFFSET);
        } else {
            setPosition(FALLBACK_NECTAR_POSITION);
        }
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
