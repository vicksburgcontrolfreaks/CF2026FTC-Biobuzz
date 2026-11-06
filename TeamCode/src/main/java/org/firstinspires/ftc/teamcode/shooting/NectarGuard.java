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
 *   3. BACKSTOP. A flywheel dip far bigger than a pollen shot's (~120 RPM on
 *      6494) while the gap is at POLLEN means something big is in the wheels.
 *      FAULT: everything off, gap OPEN to release it. Latched until reset().
 *      This is after the fact -- it limits the damage, it doesn't prevent it.
 *
 * Self-healing: a wrong trip (pollen read as nectar) costs one weak shot, and
 * that shot's dip clears it. There is deliberately NO "force pollen" button:
 * a nectar already on the ramp can't be re-detected, so clearing by hand is
 * exactly the driver error this exists to stop.
 *
 * CALIBRATION WARNING: TRIP_AMPS comes from BallLoadClassifier, measured at
 * 50 RPM indexer from rest (2026-10-30). The feed runs at
 * CollectionSystem.FEED_RPM = 25 now, so RE-RUN Indexer Load Test at the
 * feed speed and update these before trusting it. Then prove it with the
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
     * BallLoadClassifier: empty peak 0.79 + midpoint rise 0.72 = 1.51 A.
     * MEASURED AT 50 RPM -- recalibrate at the real FEED_RPM.
     */
    public static double TRIP_AMPS = 1.51;
    /**
     * Ignore this long after the indexer starts forward (motor start-up surge).
     * 0 because the 1.51 A threshold already sits above the measured empty
     * start-up peak (0.86 A max). Raise if start-up alone ever trips it.
     */
    public static double BLANK_SEC = 0.0;
    /** Forward this long and never above MIN_ALIVE_AMPS = the reading is dead. */
    public static double BLIND_AFTER_SEC = 0.5;
    public static double MIN_ALIVE_AMPS = 0.05;
    /**
     * A dip this big at the POLLEN gap = something too big is in the wheels.
     * Pollen shots dipped ~120 RPM (6494, 11-06). Set it from the shot log:
     * comfortably above the biggest real pollen dip.
     */
    public static double JAM_DIP_RPM = 400;

    // ---- State ----------------------------------------------------------------
    private State state = State.CLEAR;
    private boolean nectarRequested = false;   // operator said "nectar next"
    private double forwardSinceSec = Double.NaN;
    private boolean sawAliveThisRun = false;
    private double runPeakAmps = 0;            // peak of the current forward run
    private double lastRunPeakAmps = 0;        // frozen when the run ends -- for calibration
    private int trips = 0;
    private boolean jamArmed = false;
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
        if (jamArmed && flywheelDropRpm >= JAM_DIP_RPM) {
            state = State.FAULT;
            note = String.format("JAM? %.0f RPM dip at POLLEN gap - all stopped, gap open."
                    + " Clear it, then Y.", flywheelDropRpm);
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

        // 1. Load trip.
        if (indexerForward && forwardSec >= BLANK_SEC && indexerAmps >= TRIP_AMPS) {
            if (state != State.NECTAR_COMING) {
                trips++;
                note = String.format("NECTAR DETECTED (%.2f A) - nectar gap until it shoots", indexerAmps);
            }
            state = State.NECTAR_COMING;
            return;
        }

        // A shot clears a nectar (detected or requested): it's gone.
        if (shotSeen && (state == State.NECTAR_COMING || nectarRequested)) {
            state = State.CLEAR;
            nectarRequested = false;
            note = "nectar shot - back to pollen";
        }
    }

    /** Operator: "a nectar is next." Nectar gap until the next shot. */
    public void requestNectar() {
        nectarRequested = true;
        if (state == State.CLEAR) note = "nectar requested - nectar gap until the next shot";
    }

    /** Clear a FAULT (operator has cleared the jam). Also forgets everything else. */
    public void reset() {
        state = State.CLEAR;
        nectarRequested = false;
        forwardSinceSec = Double.NaN;
        jamArmed = false;
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
