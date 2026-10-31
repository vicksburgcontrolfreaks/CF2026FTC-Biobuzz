package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Two ways to rescue a ball that is loose inside the robot but not touching the
 * indexer -- the dead space between the collector and the indexer, where balls
 * have been getting lost.
 *
 * This is a WORKAROUND, deliberately. docs/TODO.md has carried "a MECHANICAL
 * problem in the collection path, not a code problem" since 2026-10-29, and this
 * does not fix it. It buys back a ball that would otherwise be dead weight for
 * the rest of the match. Competition robots are full of band-aids like this; the
 * mistake would be mistaking one for a repair.
 *
 * MANEUVER 1: CHASSIS SHAKE -- snap the robot forward, creep it back
 * Accelerate the chassis forward and a loose ball lags behind, so RELATIVE to
 * the robot it slides rearward, toward the indexer. Same trick as tapping a
 * ketchup bottle.
 *
 * The thing that makes or breaks it is ASYMMETRY. A plain out-and-back has four
 * acceleration phases and they cancel:
 *
 *   accelerate forward   ball shifts rearward   <- the only one we want
 *   decelerate to stop   ball shifts forward
 *   accelerate backward  ball shifts forward
 *   decelerate to stop   ball shifts rearward
 *
 * Shake it symmetrically and the ball ends up where it started. So: ONE hard
 * snap forward, above the ball's breakaway friction, then a stop and a return
 * gentle enough to stay BELOW it, so the ball rides along instead of sliding
 * back. Friction is what rectifies a back-and-forth motion into a one-way one.
 *
 * Constants.java's measured forwardZeroPowerAcceleration of -48.3 in/s^2
 * (~1.2 m/s^2, 0.12g) says the stop comes free: a ball needs roughly 3 m/s^2 to
 * break loose, so simply cutting power coasts gently enough to leave it put.
 * That is why COAST_MS exists and why the drive is forced to FLOAT -- see
 * setZeroPowerFloat() for what BRAKE would ruin.
 *
 * REPS BEAT MAGNITUDE. Peak acceleration is capped by traction, and a sliding
 * ball may only shift a few mm per snap. Repetition ratchets it; one heroic
 * lunge does not. Hence SHAKE_REPS.
 *
 * WHAT WOULD MAKE IT FAIL, in rough order of likelihood:
 *   - the ball is WEDGED, not merely resting. No chassis acceleration helps;
 *     that is geometry, not friction
 *   - the robot is pinned on a wall or being pushed, so it never accelerates.
 *     The op mode reports measured displacement so this does not look like a
 *     failed idea when it is really a blocked robot
 *   - the robot's peak acceleration does not clear the ball's breakaway. A ball
 *     free to ROLL needs almost nothing; one sliding flat on polycarb needs
 *     about as much as the drivetrain can give. Marginal, and only the robot
 *     can settle it
 *
 * MANEUVER 2: COLLECTOR PULSE -- reverse the collector, then drive it forward
 * Cheaper: nothing moves but the collector, so no field position is spent and it
 * works while pinned. Reversing briefly can unstick a ball jammed against the
 * collector and hand it a second run at the indexer.
 *
 * Its risk is the opposite of the shake's: reverse too long and the ball goes
 * out the FRONT of the robot. PULSE_REVERSE_MS is short on purpose.
 *
 * Which one wins is an empirical question. Run both from Load Recovery Test
 * against the same lost ball and keep the one that works.
 *
 * NON-BLOCKING. Call update() every loop; it returns immediately. A maneuver
 * takes 1-2 seconds, and a blocking sequence that long would freeze the drivers
 * out of their own robot -- the same mistake ShootAprilTag.alignRotationOnly
 * makes.
 */
@Configurable
public class LoadRecovery {

    // ---- Chassis shake ------------------------------------------------------
    /**
     * Full power on purpose: the ball only moves while the chassis is
     * ACCELERATING, so what matters is the jerk at the start, not the distance
     * covered. Lower this and the snap gets gentler, not shorter.
     */
    public static double SNAP_POWER = 1.0;

    /**
     * How long the snap lasts. ~100ms at a realistic 4 m/s^2 is about an inch
     * of travel and about 0.4 m/s of speed -- the inch is a side effect of a
     * burst long enough to matter, not a target in itself.
     */
    public static long SNAP_MS = 110;

    /**
     * Power cut, coasting to a stop. Must be long enough to actually stop, or
     * the return fights leftover forward speed. At the measured 48.3 in/s^2
     * coast-down, 0.4 m/s takes roughly 330ms to bleed off.
     */
    public static long COAST_MS = 350;

    /**
     * Gentle on purpose -- this is the half that must stay BELOW the ball's
     * breakaway friction so the ball rides home instead of sliding forward and
     * undoing the snap. If raising this makes the maneuver work better, the ball
     * is probably rolling rather than sliding and the asymmetry matters less
     * than expected.
     */
    public static double RETURN_POWER = 0.22;

    /**
     * Tune this against the net drift the test op mode reports. Longer if the
     * robot creeps forward over a run, shorter if it walks backward. A timed
     * return cannot be exact -- the snap slips the wheels -- so the goal is
     * drift small enough not to matter, not zero.
     */
    public static long RETURN_MS = 430;

    /** Dead time between reps, so each snap starts from genuine rest. */
    public static long SETTLE_MS = 120;

    /** Reps per press. Each one ratchets the ball back a little further. */
    public static int SHAKE_REPS = 4;

    /**
     * Flip to -1.0 if the robot snaps BACKWARD. That sends the ball toward the
     * front of the robot and possibly out of it, so CHECK THIS FIRST, at a low
     * SNAP_POWER, before running the real sequence.
     *
     * It is a live possibility, not paranoia. RobotHardware.init() sets all four
     * drive directions exactly INVERTED from pedroPathing/Constants.java, and
     * Constants wins only because createFollower() runs afterward. Raw motor
     * power -- which is what this class uses -- therefore depends on the
     * follower having been built. On top of that, drive directions were only
     * ever verified on the 2025 robot (connor/drive-motor-directions).
     */
    public static double SNAP_SIGN = 1.0;

    // ---- Collector pulse ----------------------------------------------------
    public static double PULSE_POWER = 0.6;
    /** Short. Long enough to unstick, too short to spit the ball out the front. */
    public static long PULSE_REVERSE_MS = 130;
    public static long PULSE_FORWARD_MS = 260;
    public static int PULSE_REPS = 3;

    // ---- Shared ------------------------------------------------------------
    /**
     * The indexer runs through both maneuvers, so a freed ball gets grabbed the
     * instant it makes contact. 50 RPM matches BallLoadClassifier's calibration,
     * so a ball recovered here can still be classified on the way up.
     */
    public static double INDEXER_RPM = 50.0;

    /** Collector runs forward during the shake, nudging the ball rearward too. */
    public static boolean COLLECTOR_ON_DURING_SHAKE = true;
    public static double COLLECTOR_SHAKE_POWER = 0.5;

    public enum State {
        IDLE,
        /** Hard forward burst. The only phase that is supposed to move the ball. */
        SNAP,
        /** Power cut, coasting down gently enough to leave the ball put. */
        COAST,
        /** Low-power reverse, back toward where the rep started. */
        RETURN,
        /** At rest between reps. */
        SETTLE,
        /** Collector backing up. */
        PULSE_REVERSE,
        /** Collector driving in. */
        PULSE_FORWARD
    }

    private final DcMotorEx lf, lr, rf, rr;
    private final DcMotorEx collector;
    private final DcMotorEx indexer;

    private final ElapsedTime phaseTimer = new ElapsedTime();

    private State state = State.IDLE;
    private int repsDone = 0;
    private int repsTarget = 0;
    private boolean shaking = false;

    private DcMotor.ZeroPowerBehavior savedZeroPower = null;

    public LoadRecovery(DcMotorEx lf, DcMotorEx lr, DcMotorEx rf, DcMotorEx rr,
                        DcMotorEx collector, DcMotorEx indexer) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.collector = collector;
        this.indexer = indexer;
    }

    // ---- Starting and stopping ---------------------------------------------

    /** Begins the chassis shake. Ignored if a maneuver is already running. */
    public void startShake() {
        if (state != State.IDLE) {
            return;
        }
        setZeroPowerFloat();
        repsDone = 0;
        repsTarget = Math.max(1, SHAKE_REPS);
        shaking = true;
        enter(State.SNAP);
    }

    /** Begins the collector pulse. Ignored if a maneuver is already running. */
    public void startCollectorPulse() {
        if (state != State.IDLE) {
            return;
        }
        repsDone = 0;
        repsTarget = Math.max(1, PULSE_REPS);
        shaking = false;
        enter(State.PULSE_REVERSE);
    }

    /** Stops everything this class commands, immediately. */
    public void cancel() {
        stopDrive();
        collector.setPower(0.0);
        indexer.setPower(0.0);
        restoreZeroPower();
        state = State.IDLE;
        // shaking is left alone on purpose -- it describes the current OR last
        // maneuver, so telemetry can still say which one just ran.
    }

    // ---- The state machine -------------------------------------------------

    /**
     * Call every loop. Returns immediately.
     *
     * CALL THIS AFTER follower.update(), NOT BEFORE. Pedro's drivetrain may
     * write zero powers while no path is being followed, so whichever runs last
     * wins the motors. Swap the order and the snap silently does nothing.
     */
    public void update() {
        if (state == State.IDLE) {
            return;
        }

        indexer.setVelocity(rpmToTicksPerSec(INDEXER_RPM));

        // Commanded every loop rather than once at the start of SNAP, so the
        // collector keeps feeding through the coast and the return too. Setting
        // it in one phase and relying on the motor to hold that value works, but
        // it hides when the collector is running.
        if (shaking) {
            collector.setPower(
                    COLLECTOR_ON_DURING_SHAKE ? COLLECTOR_SHAKE_POWER : 0.0);
        }

        switch (state) {
            case SNAP:
                driveForward(SNAP_POWER);
                if (phaseTimer.milliseconds() >= SNAP_MS) {
                    enter(State.COAST);
                }
                break;

            case COAST:
                // Zero power, FLOAT: the chassis slows at its own ~48 in/s^2,
                // which is below what the ball needs to break loose. Braking
                // here would decelerate hard enough to throw the ball forward
                // again and cancel the snap.
                stopDrive();
                if (phaseTimer.milliseconds() >= COAST_MS) {
                    enter(State.RETURN);
                }
                break;

            case RETURN:
                driveForward(-RETURN_POWER);
                if (phaseTimer.milliseconds() >= RETURN_MS) {
                    enter(State.SETTLE);
                }
                break;

            case SETTLE:
                stopDrive();
                if (phaseTimer.milliseconds() >= SETTLE_MS) {
                    repsDone++;
                    if (repsDone >= repsTarget) {
                        finish();
                    } else {
                        enter(State.SNAP);
                    }
                }
                break;

            case PULSE_REVERSE:
                collector.setPower(-PULSE_POWER);
                if (phaseTimer.milliseconds() >= PULSE_REVERSE_MS) {
                    enter(State.PULSE_FORWARD);
                }
                break;

            case PULSE_FORWARD:
                collector.setPower(PULSE_POWER);
                if (phaseTimer.milliseconds() >= PULSE_FORWARD_MS) {
                    repsDone++;
                    if (repsDone >= repsTarget) {
                        finish();
                    } else {
                        enter(State.PULSE_REVERSE);
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    private void enter(State next) {
        state = next;
        phaseTimer.reset();
    }

    private void finish() {
        stopDrive();
        collector.setPower(0.0);
        indexer.setPower(0.0);
        restoreZeroPower();
        state = State.IDLE;
    }

    // ---- Drive plumbing ----------------------------------------------------

    /**
     * All four wheels at the same power, which is straight forward for this
     * mecanum -- the same mixing DriveTeleOp uses with x and rx at zero.
     */
    private void driveForward(double power) {
        double p = power * SNAP_SIGN;
        lf.setPower(p);
        lr.setPower(p);
        rf.setPower(p);
        rr.setPower(p);
    }

    private void stopDrive() {
        lf.setPower(0.0);
        lr.setPower(0.0);
        rf.setPower(0.0);
        rr.setPower(0.0);
    }

    /**
     * Forces FLOAT so the coast phase actually coasts. On BRAKE the hub shorts
     * the motor windings and the robot stops far harder than the 48.3 in/s^2
     * Constants measured -- hard enough to slide the ball forward and undo the
     * snap. The original behavior is restored when the maneuver ends.
     *
     * Deliberately does NOT touch run modes. lf carries the turret's Through
     * Bore encoder, and RobotHardware.init() spells out that lf must stay in
     * RUN_WITHOUT_ENCODER or the hub tries to regulate a drive wheel against the
     * TURRET's position. RUN_WITHOUT_ENCODER is what the snap wants anyway --
     * raw duty cycle, no velocity PID braking against us.
     */
    private void setZeroPowerFloat() {
        if (savedZeroPower == null) {
            savedZeroPower = lf.getZeroPowerBehavior();
        }
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void restoreZeroPower() {
        if (savedZeroPower == null) {
            return;
        }
        lf.setZeroPowerBehavior(savedZeroPower);
        lr.setZeroPowerBehavior(savedZeroPower);
        rf.setZeroPowerBehavior(savedZeroPower);
        rr.setZeroPowerBehavior(savedZeroPower);
        savedZeroPower = null;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * RobotHardware.TICKS_PER_MOTOR_REV
                * RobotHardware.INDEXER_GEAR_RATIO / 60.0;
    }

    // ---- Readouts ----------------------------------------------------------

    public boolean isRunning() {
        return state != State.IDLE;
    }

    public State getState() {
        return state;
    }

    /** True if the running -- or most recently run -- maneuver is the shake. */
    public boolean isShaking() {
        return shaking;
    }

    public int getRepsDone() {
        return repsDone;
    }

    public int getRepsTarget() {
        return repsTarget;
    }

    /** One line for telemetry: what it is doing and how far along. */
    public String getStatus() {
        if (state == State.IDLE) {
            return "idle";
        }
        return (shaking ? "SHAKE" : "PULSE") + " " + state
                + "  rep " + (repsDone + 1) + "/" + repsTarget;
    }
}
