package org.firstinspires.ftc.teamcode.shooting;

/*
 * ShooterLogic: the shooter's brain, with no hardware in it.
 *
 * The old ShootAprilTag.shoot() used sleep()s: spin up, wait 1 s, feed, wait...
 * While it sleeps, NOTHING else runs -- no driving, no turret, no telemetry, and
 * in auto, no path following. So instead this is a STATE MACHINE: every loop,
 * update() looks at the time and the wheel speeds, decides what to do right now,
 * and returns immediately.
 *
 *   IDLE --spinUp()--> SPINNING_UP --both wheels at speed--> READY
 *                                                              |
 *                     +------------------ fire(n) ------------+
 *                     v
 *                  FEEDING --FEED_SEC--> RECOVERING --back at speed--> FEEDING (more shots)
 *                                                                  --> READY   (done)
 *
 * "AT SPEED" means BOTH wheels within TOLERANCE_RPM of the target, continuously,
 * for SETTLE_SEC. A wheel shooting past the target on the way up is in tolerance
 * for a moment too -- that's not ready, and feeding then is how you get one good
 * shot and two short ones.
 *
 * Each shot steals speed from the wheels (watch getLastDipRpm()).
 * RECOVERING waits for them to come back before the next piece goes in.
 *
 * The real class, subsystems/Shooter, reads the motors, calls update(), and
 * writes what this says. Everything here is tested on a laptop (ShooterLogicTest).
 */
public class ShooterLogic {

    /** How the indexer pushes pieces in. TODO(on robot): nobody knows which yet. */
    public enum FeedMode {
        /** Run the indexer FEED_SEC per piece, then wait for the wheels to recover. */
        PULSE,
        /** Run the indexer non-stop for CONTINUOUS_SEC_PER_PIECE x pieces. No recovery waits. */
        CONTINUOUS
    }

    public enum State { IDLE, SPINNING_UP, READY, FEEDING, RECOVERING }

    // ---- Tuning (starting guesses -- tune on the robot) ---------------------
    public static double TOLERANCE_RPM = 50;
    public static double SETTLE_SEC = 0.15;
    public static FeedMode FEED_MODE = FeedMode.PULSE;
    public static double FEED_POWER = 1.0;
    public static double FEED_SEC = 0.25;
    public static double CONTINUOUS_SEC_PER_PIECE = 0.35;
    /** Shortest pause between pulses, even if the wheels say they're back already. */
    public static double MIN_RECOVERY_SEC = 0.10;
    /** Flag it if spin-up takes longer than this. Keeps spinning; the caller decides. */
    public static double SPINUP_WARN_SEC = 2.0;
    /** Flag it if the two wheels differ by more than this (cross-hub skew, a loose belt...). */
    public static double DISAGREE_RPM = 100;

    private State state = State.IDLE;
    private double targetRpm = 0;
    private int shotsQueued = 0;
    private int shotsFired = 0;

    private double stateStartSec = 0;
    private double inToleranceSinceSec = Double.NaN;   // NaN = not in tolerance right now
    private double feedSecThisRun = 0;

    private double dipLowRpm = Double.NaN;   // lowest wheel speed since the current shot started
    private double lastDipRpm = 0;
    private boolean spinUpSlow = false;
    private boolean wheelsDisagree = false;
    private String note = "idle";

    // ---- Commands -----------------------------------------------------------

    /** Spin both wheels to this RPM. Also call it every loop to follow a changing distance. */
    public void spinUp(double rpm) {
        targetRpm = Math.max(0, rpm);
        if (state == State.IDLE && targetRpm > 0) {
            state = State.SPINNING_UP;
            stateStartSec = Double.NaN;   // stamped on the next update()
            inToleranceSinceSec = Double.NaN;
            spinUpSlow = false;
        }
    }

    /**
     * Queue n pieces. They go as soon as the wheels are ready. Ignored while
     * IDLE -- spinUp() first, so a stray button press can't feed a stopped shooter.
     */
    public void fire(int n) {
        if (state == State.IDLE || n <= 0) return;
        shotsQueued += n;
    }

    /** Wheels off, indexer off, queued shots dropped. */
    public void stop() {
        state = State.IDLE;
        targetRpm = 0;
        shotsQueued = 0;
        note = "idle";
    }

    // ---- The loop -----------------------------------------------------------

    /** Run every loop. Times in seconds (any clock that only goes up), speeds in wheel RPM. */
    public void update(double nowSec, double leftRpm, double rightRpm) {
        if (Double.isNaN(stateStartSec)) stateStartSec = nowSec;
        wheelsDisagree = state != State.IDLE && Math.abs(leftRpm - rightRpm) > DISAGREE_RPM;

        boolean inTol = Math.abs(leftRpm - targetRpm) <= TOLERANCE_RPM
                && Math.abs(rightRpm - targetRpm) <= TOLERANCE_RPM;
        if (!inTol) {
            inToleranceSinceSec = Double.NaN;
        } else if (Double.isNaN(inToleranceSinceSec)) {
            inToleranceSinceSec = nowSec;
        }
        boolean atSpeed = inTol && nowSec - inToleranceSinceSec >= SETTLE_SEC;

        if (state == State.FEEDING || state == State.RECOVERING) {
            double low = Math.min(leftRpm, rightRpm);
            dipLowRpm = Double.isNaN(dipLowRpm) ? low : Math.min(dipLowRpm, low);
        }

        double inState = nowSec - stateStartSec;
        switch (state) {
            case IDLE:
                break;

            case SPINNING_UP:
                if (atSpeed) {
                    enter(State.READY, nowSec);
                    // fall into READY next loop; one loop of delay is fine
                } else {
                    spinUpSlow = inState > SPINUP_WARN_SEC;
                    note = spinUpSlow ? "SLOW SPIN-UP - can it reach " + (int) targetRpm + "?" : "spinning up";
                }
                break;

            case READY:
                if (!atSpeed) {
                    enter(State.SPINNING_UP, nowSec);   // target changed, or something slowed the wheels
                } else if (shotsQueued > 0) {
                    startFeed(nowSec);
                } else {
                    note = "ready";
                }
                break;

            case FEEDING:
                if (inState >= feedSecThisRun) {
                    if (FEED_MODE == FeedMode.CONTINUOUS) {
                        shotsFired += shotsQueued;
                        shotsQueued = 0;
                        finishShot();
                        enter(State.SPINNING_UP, nowSec);   // re-check speed before calling it ready
                    } else {
                        shotsQueued--;
                        shotsFired++;
                        enter(State.RECOVERING, nowSec);
                    }
                }
                break;

            case RECOVERING:
                if (inState >= MIN_RECOVERY_SEC && atSpeed) {
                    finishShot();
                    if (shotsQueued > 0) {
                        startFeed(nowSec);
                    } else {
                        enter(State.READY, nowSec);
                    }
                } else {
                    note = "recovering";
                }
                break;
        }
    }

    private void startFeed(double nowSec) {
        feedSecThisRun = FEED_MODE == FeedMode.CONTINUOUS
                ? CONTINUOUS_SEC_PER_PIECE * shotsQueued
                : FEED_SEC;
        dipLowRpm = Double.NaN;
        enter(State.FEEDING, nowSec);
        note = "feeding";
    }

    private void finishShot() {
        if (!Double.isNaN(dipLowRpm)) lastDipRpm = targetRpm - dipLowRpm;
    }

    private void enter(State next, double nowSec) {
        state = next;
        stateStartSec = nowSec;
        spinUpSlow = false;
        if (next == State.READY) note = "ready";
        if (next == State.RECOVERING) note = "recovering";
        if (next == State.SPINNING_UP) note = "spinning up";
    }

    // ---- What to write to the hardware --------------------------------------

    /** Both wheels' target, RPM. 0 when IDLE. */
    public double getTargetRpm() {
        return state == State.IDLE ? 0 : targetRpm;
    }

    /**
     * True only while this class is driving the indexer. When false, LEAVE THE
     * INDEXER ALONE -- the driver may be using it to unjam.
     */
    public boolean ownsIndexer() {
        return state == State.FEEDING;
    }

    /** Indexer power while ownsIndexer(). */
    public double getIndexerPower() {
        return ownsIndexer() ? FEED_POWER : 0;
    }

    // ---- Readouts -----------------------------------------------------------

    public State getState() { return state; }
    /** Wheels at speed and nothing queued or in flight. */
    public boolean isReady() { return state == State.READY && shotsQueued == 0; }
    /** Shots still queued or in flight. Auto: fire(3), then wait until this is false. */
    public boolean isBusy() { return shotsQueued > 0 || state == State.FEEDING || state == State.RECOVERING; }
    public int getShotsQueued() { return shotsQueued; }
    public int getShotsFired() { return shotsFired; }
    /** How far the wheels sagged during the last shot, RPM. A shot that didn't dip probably didn't happen. */
    public double getLastDipRpm() { return lastDipRpm; }
    public boolean isSpinUpSlow() { return spinUpSlow; }
    public boolean doWheelsDisagree() { return wheelsDisagree; }
    public String getNote() { return wheelsDisagree ? note + " | WHEELS DISAGREE" : note; }
}
