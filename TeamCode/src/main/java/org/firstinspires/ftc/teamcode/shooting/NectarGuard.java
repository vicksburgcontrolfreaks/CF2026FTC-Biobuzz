package org.firstinspires.ftc.teamcode.shooting;

/*
 * NectarGuard: keeps a nectar ball from ever meeting the pollen-width gap.
 *
 * WHY: we run the pivot at the POLLEN gap (we're playing pollen), and nobody can
 * see which ball is next inside the robot. A nectar into the pollen gap is
 * damage -- exactly what, we don't know, but nothing good. So: SHOOT IT SAFELY.
 *
 * HOW (three layers, most useful first):
 *
 *   1. LOAD TRIP. A nectar is bigger, so the indexer works harder pushing it:
 *      its current jumps (BallLoadClassifier has the measurements). While the
 *      indexer runs FORWARD, current above TRIP_AMPS = nectar -> open the gap to
 *      NECTAR right away and keep it there until that ball's shot is seen (the
 *      flywheel dip), then back to pollen. There is time: ~6 in. of ramp between
 *      indexer and wheels, 0.6-1.2 s at 50 RPM indexer (longer at 25), against a
 *      0.3-0.5 s servo move. NOTE stopping the indexer would NOT help -- once the
 *      ball is on the ramp it rolls on its own. Moving the pivot does.
 *
 *   2. BLIND CHECK. If the indexer has run forward a while and the current never
 *      rose above MIN_ALIVE_AMPS, the reading is broken and layer 1 is blind.
 *      Fail SAFE: nectar gap (pollen shots get weak, nothing breaks) + alert.
 *
 *   3. BACKSTOP -- OFF by default (BACKSTOP_ENABLED; bumps tripped it).
 *      A flywheel dip far bigger than a pollen shot's (~120 RPM on
 *      6494) while the gap is at POLLEN means something big is in the wheels.
 *      FAULT: flywheel + indexer off (collector keeps going), gap OPEN to
 *      release it. Latched until reset().
 *      This is after the fact -- it limits the damage, it doesn't prevent it.
 *
 * Recovering from a wrong trip (pollen read as nectar): at the nectar gap a
 * pollen is NOT gripped -- it sticks in the opening (6494, 2026-11-06), so there
 * is no dip to clear it. STUCK_CLEAR_SEC handles that: wide gap, wheels at
 * speed, indexer pushing, nothing shot -> not a nectar -> back to pollen, which
 * grips and shoots it. There is deliberately NO "force pollen" button:
 * a nectar already on the ramp can't be re-detected, so clearing by hand is
 * exactly the driver error this exists to stop.
 *
 * CALIBRATION: TRIP_AMPS was measured at FEED_RPM 25 (2026-11-06), indexer
 * alone. Still to check: pollen pushed by the COLLECTOR too (RT), which adds
 * load the load test doesn't. Then prove it with the
 * flywheels OFF: 10 nectars must open the gap before reaching the wheels,
 * 10 pollens must not.
 *
 * Plain numbers, no hardware: NectarGuardTest runs it on the laptop.
 */
public class NectarGuard {

    /** What gap the pivot should be at. */
    public enum Gap { POLLEN, NECTAR, OPEN }

    public enum State {
        /** Pollen gap, watching. */
        CLEAR,
        /** A nectar is (probably) on its way: nectar gap until its shot is seen. */
        NECTAR_COMING,
        /** Current reading looks dead: nectar gap until it reads again. */
        BLIND,
        /** Something big hit the pollen gap. Everything off, gap open. reset() to clear. */
        FAULT
    }

    // ---- Tuning ---------------------------------------------------------------
    /**
     * Indexer current (A, normalized to 12 V) that means nectar.
     *
     * RECALIBRATED 2026-11-06 on 6494, Indexer Load Test at FEED_RPM 25, indexer
     * alone (no collector), peak per pass:
     *   empty   0.48 / 0.54 / 0.58 A        (min / mean / max)
     *   pollen  0.75 / 1.19 / 1.65 A        (rise 0.21 / 0.65 / 1.11)
     *   nectar  3.79 / 4.97 / 7.67 A        (rise 3.25 / 4.43 / 7.13)
     * Gap 2.14 A (wider than 50 RPM's 1.32); threshold at the midpoint, ~1.07 A
     * margin each way. The old 1.51 (from the 50 RPM data) sat BELOW the heaviest
     * pollen at 25 -- that was the false trips.
     *
     * NOT the same number as BallLoadClassifier's: that one is the 50 RPM
     * calibration and Load Sort Test / LoadRecovery still run at 50.
     * Change FEED_RPM -> re-run Indexer Load Test -> update this.
     */
    public static double TRIP_AMPS = 2.72;
    /**
     * Ignore this long after the indexer starts forward (motor start-up surge).
     * 0 because TRIP_AMPS (2.72) sits far above the measured empty start-up
     * peak (0.58 A max at 25 RPM, 0.86 at 50). Raise if start-up alone ever trips it.
     */
    public static double BLANK_SEC = 0.0;
    /**
     * Current must stay at/above TRIP_AMPS this long to count. A nectar pushes
     * hard the whole time it's squeezed through; a start-up surge or a ball
     * shifting is a blip. Added 2026-11-06 after false trips on 6494.
     */
    public static double TRIP_HOLD_SEC = 0.10;
    /**
     * STUCK RECOVERY. A nectar at the nectar gap gets gripped and shot. So if
     * the gap has been wide, the flywheel at speed, and the indexer pushing for
     * this long with NO shot, what's there isn't a nectar -- it's a pollen the
     * wide gap can't grip (seen on 6494, 2026-11-06: it just sat in the
     * opening). Close to pollen so it shoots. The jam backstop still covers a
     * wrong call. ~2x the slowest ramp travel at FEED_RPM 25.
     */
    public static double STUCK_CLEAR_SEC = 4.0;
    /** "At speed" for STUCK_CLEAR_SEC: the slower wheel within this of target. */
    public static double STUCK_AT_SPEED_RPM = 100;
    /** Forward this long and never above MIN_ALIVE_AMPS = the reading is dead. */
    public static double BLIND_AFTER_SEC = 0.5;
    public static double MIN_ALIVE_AMPS = 0.05;
    /**
     * A dip this big at the POLLEN gap = something too big is in the wheels.
     * Pollen shots dipped ~120 RPM (6494, 11-06). Set it from the shot log:
     * comfortably above the biggest real pollen dip.
     */
    public static double JAM_DIP_RPM = 400;
    /**
     * OFF (2026-11-06, Shane): on 6494 it kept firing when the robot bumped into
     * things -- a hit can sag the wheels or the battery enough to look like a
     * jam -- and stopping everything mid-drive was worse than what it protects
     * against. The LOAD TRIP (layer 1) stays on. Before turning this back on, get
     * the real biggest pollen dip from the shot log and check bumps don't trip it.
     */
    public static boolean BACKSTOP_ENABLED = false;
    /** The big dip must last this long to count (a bump is a blip; a jam stays). */
    public static double JAM_HOLD_SEC = 0.15;

    // ---- State ----------------------------------------------------------------
    private State state = State.CLEAR;
    private boolean nectarRequested = false;   // operator said "nectar next"
    private double forwardSinceSec = Double.NaN;
    private boolean sawAliveThisRun = false;
    private double runPeakAmps = 0;            // peak of the current forward run
    private double lastRunPeakAmps = 0;        // frozen when the run ends -- for calibration
    private int trips = 0;
    private boolean jamArmed = false;
    private double jamSinceSec = Double.NaN;
    private double aboveSinceSec = Double.NaN;   // current at/above TRIP_AMPS since, NaN = below
    private boolean tripArmed = true;            // false after a trip until current drops below again
    private double stuckSec = 0;                 // wide gap + at speed + pushing, no shot
    private double lastNowSec = Double.NaN;
    private String note = "clear";

    /**
     * Run every loop.
     *
     * @param nowSec          any clock that only goes up
     * @param indexerForward  the indexer is being driven FORWARD (toward the wheels) right now
     * @param indexerAmps     indexer current, normalized to 12 V (only looked at while forward)
     * @param shotSeen        the shooter counted a shot (flywheel dip) THIS loop
     * @param flywheelDropRpm target minus the slower wheel; NaN when the flywheel is off
     */
    public void update(double nowSec, boolean indexerForward, double indexerAmps,
                       boolean shotSeen, double flywheelDropRpm) {
        double dt = Double.isNaN(lastNowSec) ? 0 : nowSec - lastNowSec;
        lastNowSec = nowSec;
        if (state == State.FAULT) return;   // latched; only reset() clears it

        // 3. Backstop first: if the gap is at pollen and the wheels just took a
        //    huge hit, it's too late for anything else.
        //    ARMED only once the wheels are near speed AT the pollen gap. Without
        //    that, spin-up (drop = the whole target) and the recovery right after
        //    a nectar shot (gap just closed, wheels still low) would both trip it.
        boolean flywheelOn = !Double.isNaN(flywheelDropRpm);
        if (!flywheelOn || gap() != Gap.POLLEN) {
            jamArmed = false;
        } else if (flywheelDropRpm < JAM_DIP_RPM / 2) {
            jamArmed = true;
        }
        boolean bigDip = jamArmed && flywheelDropRpm >= JAM_DIP_RPM;
        if (!bigDip) {
            jamSinceSec = Double.NaN;
        } else if (Double.isNaN(jamSinceSec)) {
            jamSinceSec = nowSec;
        }
        if (BACKSTOP_ENABLED && bigDip && nowSec - jamSinceSec >= JAM_HOLD_SEC) {
            state = State.FAULT;
            note = String.format("JAM? %.0f RPM dip at POLLEN gap - flywheel + indexer stopped,"
                    + " gap open. Clear it, then Y.", flywheelDropRpm);
            return;
        }

        // Track this forward run.
        if (indexerForward) {
            if (Double.isNaN(forwardSinceSec)) {
                forwardSinceSec = nowSec;
                sawAliveThisRun = false;
                runPeakAmps = 0;
            }
            runPeakAmps = Math.max(runPeakAmps, indexerAmps);
            if (indexerAmps > MIN_ALIVE_AMPS) sawAliveThisRun = true;
        } else if (!Double.isNaN(forwardSinceSec)) {
            lastRunPeakAmps = runPeakAmps;
            forwardSinceSec = Double.NaN;
        }
        double forwardSec = indexerForward ? nowSec - forwardSinceSec : 0;

        // 2. Blind: running forward a while, current never came alive.
        if (indexerForward && forwardSec >= BLIND_AFTER_SEC && !sawAliveThisRun) {
            if (state != State.BLIND) {
                state = State.BLIND;
                note = "LOAD SENSOR BLIND (no indexer current) - nectar gap, shots may be weak";
            }
            return;
        }
        if (state == State.BLIND && sawAliveThisRun) {
            state = State.CLEAR;
            note = "load sensor reading again";
        }

        // 1. Load trip: at/above TRIP_AMPS for TRIP_HOLD_SEC straight. Re-arms
        //    only once the current drops below again, so one long hard push
        //    (e.g. a stuck ball) can't trip again and again.
        boolean above = indexerForward && forwardSec >= BLANK_SEC && indexerAmps >= TRIP_AMPS;
        if (!above) {
            aboveSinceSec = Double.NaN;
            tripArmed = true;
        } else if (Double.isNaN(aboveSinceSec)) {
            aboveSinceSec = nowSec;
        }
        if (above && tripArmed && nowSec - aboveSinceSec >= TRIP_HOLD_SEC) {
            tripArmed = false;
            trips++;
            stuckSec = 0;
            state = State.NECTAR_COMING;
            note = String.format("NECTAR DETECTED (%.2f A) - nectar gap until it shoots", indexerAmps);
            return;
        }

        boolean wide = state == State.NECTAR_COMING || (state == State.CLEAR && nectarRequested);

        // A shot clears a nectar (detected or requested): it's gone.
        if (shotSeen && wide) {
            state = State.CLEAR;
            nectarRequested = false;
            stuckSec = 0;
            note = "nectar shot - back to pollen";
            return;
        }

        // Stuck recovery (see STUCK_CLEAR_SEC).
        if (wide && indexerForward && flywheelOn && flywheelDropRpm < STUCK_AT_SPEED_RPM) {
            stuckSec += dt;
            if (stuckSec >= STUCK_CLEAR_SEC) {
                state = State.CLEAR;
                nectarRequested = false;
                stuckSec = 0;
                note = String.format("nothing shot in %.0f s at the nectar gap - not a nectar,"
                        + " back to pollen", STUCK_CLEAR_SEC);
            }
        } else if (!wide) {
            stuckSec = 0;
        }
    }

    /** Operator: "a nectar is next." Nectar gap until the next shot. */
    public void requestNectar() {
        if (!nectarRequested) stuckSec = 0;
        nectarRequested = true;
        if (state == State.CLEAR) note = "nectar requested - nectar gap until the next shot";
    }

    /** Clear a FAULT (operator has cleared the jam). Also forgets everything else. */
    public void reset() {
        state = State.CLEAR;
        nectarRequested = false;
        forwardSinceSec = Double.NaN;
        jamArmed = false;
        jamSinceSec = Double.NaN;
        aboveSinceSec = Double.NaN;
        tripArmed = true;
        stuckSec = 0;
        note = "clear";
    }

    // ---- Outputs --------------------------------------------------------------

    /** Where the pivot should be right now. */
    public Gap gap() {
        switch (state) {
            case FAULT:
                return Gap.OPEN;
            case NECTAR_COMING:
            case BLIND:
                return Gap.NECTAR;
            default:
                return nectarRequested ? Gap.NECTAR : Gap.POLLEN;
        }
    }

    /** True = stop the flywheels, indexer and collector, now. */
    public boolean isFault() { return state == State.FAULT; }

    public State getState() { return state; }
    public String getNote() { return note; }
    /** How many times the load trip has fired since INIT. */
    public int getTrips() { return trips; }
    /** Peak indexer amps of the last finished forward run -- read this to recalibrate. */
    public double getLastRunPeakAmps() { return lastRunPeakAmps; }
    /** Peak so far in the current forward run. */
    public double getRunPeakAmps() { return runPeakAmps; }
}
