package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.teamcode.vision.TurretCamera;

import java.util.ArrayList;
import java.util.List;

/*
 * Drives any Route, for either alliance. Every route auto uses this same
 * code; the route itself comes from Routes.
 *
 * SelectAuto is the one Driver Station entry: the drive team picks the
 * alliance and route on the gamepad during INIT, and it calls useRoute().
 * (A tiny file can also pass a fixed alliance and route to the constructor
 * -- see the old RedStartUp etc. in legacy/.)
 *
 *   1. Start at the route's start (after WAIT_SECONDS, 0 = go right away)
 *   2. Drive to each step's point, stopping at each one
 *      - COLLECT steps: collector on once within COLLECT_RADIUS of the
 *        point, off when we get there
 *      - LAUNCH steps: on arrival, wait for the turret to aim (AIM_WAIT_SEC
 *        max), then collector + indexer + flywheels for LAUNCH_MAX_SEC (or, with
 *        LAUNCH_FULL_TIME off, until LAUNCH_SHOTS / no ball), move on.
 *        The flywheel pre-spins from Start; the turret aims by camera the
 *        whole way at our cell on our side of the field.
 *      - YIELD steps: on arrival, hold still until yieldUntilSeconds
 *   3. Say "Done" on the Driver Station, and show how long the run took
 *
 * "abstract" means you can't run this class by itself -- only an OpMode
 * built on it, like SelectAuto.
 */
public abstract class RouteAuto extends OpMode {

    // On a COLLECT step, turn the collector on when the robot gets this close
    // (inches) to the step's point.
    private static final double COLLECT_RADIUS = 22;

    // How long to wait before starting, for the tiny files. 0 = go right away.
    // (SelectAuto picks its own wait on the gamepad.)
    private static final double WAIT_SECONDS = 0;

    // ---- Launching (real, 2026-11-06) ----------------------------------------
    // A LAUNCH step: on arrival, wait for the turret to be on target (up to
    // AIM_WAIT_SEC -- after that, fire anyway: a miss costs no more than not
    // shooting), then fire until LAUNCH_SHOTS dips are counted, the shooter says
    // "no ball", or LAUNCH_MAX_SEC runs out. Starting guesses -- time real runs.
    public static int LAUNCH_SHOTS = 4;          // the 4 preloads (Routes.gardenWallLaunch)
    public static double AIM_WAIT_SEC = 2.0;
    /** Seconds of firing (collector + indexer + flywheels), counted from when firing starts. */
    public static double LAUNCH_MAX_SEC = 15.0;   // 8 -> 12 -> 15: still short on shots (Shane, 2026-11-06)
    /**
     * true = always fire the full LAUNCH_MAX_SEC, ignoring the shot count and
     * "no ball" (Shane, 2026-11-06: certain to empty the robot; dip counting
     * isn't trusted yet). false = stop early at LAUNCH_SHOTS or "no ball".
     */
    public static boolean LAUNCH_FULL_TIME = true;

    /**
     * Home the deflector and turret during INIT instead of at Start.
     *
     * BACK ON (2026-11-10), after a clean standalone run: Deflector Auto Home
     * found HOME, anchored the zero to it and climbed to the 35 stow, resting at
     * 35.45.
     *
     * It was off briefly because it mis-pointed the deflector twice: a stall with
     * NO TRAVEL was read as "on the stop", the zero was set at the stow, and it
     * climbed 46.9 deg from there -- cumulatively, across two runs.
     * Deflector.HOME_MIN_TRAVEL_SERVO_DEG now refuses that, and the clean run
     * showed it does not block a real seek.
     *
     * Worth knowing why that was the only hazard: the seek sets the zero from
     * total AT THE STOP, so a turn count corrupted beforehand (by someone moving
     * the deflector by hand, say) is simply overwritten by a physical reference.
     * The seek is self-correcting for anything except believing it has arrived
     * when it never moved.
     *
     * Also still unconfirmed: whether the turret.s 150 deg sweep stays inside
     * the 18 in. starting cube during INIT. See the note in init().
     */
    public static boolean AUTO_HOME_IN_INIT = true;

    // The shot arc lives in Deflector.DEFAULT_SHOT_DEG now -- one number shared by
    // auto, TeleOp and the tuner, so the three cannot drift apart. Was AUTO_ARC_DEG
    // here at 70 (67.5 came up short, Shane 2026-11-06); 67 is the current value,
    // re-found against the 11.9 hard-stop zero.

    // ---- TIP_LAUNCH (Far Wall Tip Shoot + Park, 2026-11-08) -------------------
    /**
     * Seconds after Start when a TIP_LAUNCH step gives up -- stops waiting for the
     * tip, or stops firing -- and drives to the park. The park leg from FAR_SHOT
     * is ~43 in. plus a ~72 deg turn; 25 leaves ~5 s for it. TIME THE PARK LEG on
     * the robot and raise this if it gets there with time to spare.
     */
    public static double PARK_LEAVE_BY_SEC = 25.0;
    /** Camera frames with one of our cell's tags in them before we believe the hive tipped. */
    public static int TIP_CONFIRM_FRAMES = 3;

    // 1.0 = full speed (Shane, 2026-11-06). Was 0.5 while 6494's odometry drifts
    // ~1" per round trip (docs/TODO.md item 0) -- at full speed that drift and
    // braking overshoot both grow, so retest every route on both robots.
    // Back to 1.0 after Garden Shoot + Park ran clean at 0.5 (Shane, 2026-11-06).
    private static final double MAX_POWER = 1.0;

    private Alliance alliance;
    private Route route;
    private double waitSeconds = WAIT_SECONDS;

    private Follower follower;
    private Timer pathTimer;
    private boolean collectorOn = false;

    // Same pieces as DriveTeleOp, so the nectar guard and at-speed gate come along.
    private Launcher launcher;
    private final Launcher.Controls controls = new Launcher.Controls();
    private Turret turret;
    private TurretCamera camera;
    private TurretAim aim;
    private FieldAprilTags.Cell aimCell;
    private Deflector deflector;

    private Timer launchTimer;
    private boolean launching = false;
    private boolean launchFiring = false;   // aim wait over, shots going
    private double fireStartSec = 0;         // launchTimer time when firing started
    private boolean tipSeen = false;         // TIP_LAUNCH: our cell's tags confirmed
    private int tipFrames = 0;               // camera frames with our cell's tags in them
    private long lastTipFrame = 0;           // so one frame seen over several loops counts once

    // End of route: turret to the magnet, then off until TeleOp (see stowTurret).
    public static double STOW_MAX_SEC = 3.0;
    private Timer stowTimer = null;
    private boolean turretStowed = false;
    private int shotsAtLaunchStart = 0;
    private String launchNote = "";

    private Timer runTimer;               // seconds since Start
    private double finishedSeconds = -1;  // when the last step finished, -1 = not yet

    // A YIELD step holds still until this many seconds after Start.
    // 0 = don't wait at all. SelectAuto sets it from the menu.
    private double yieldUntilSeconds = 0;
    private boolean yielding = false;     // just for telemetry

    private Pose startPose;                              // for OUR alliance
    private final List<Pose> targets = new ArrayList<>(); // targets.get(i) = where step i ends, for OUR alliance
    private final List<Path> paths = new ArrayList<>();   // paths.get(i) = how step i gets there

    // Which step we're on. -1 = waiting at the start, steps.size() = done.
    private int step = -1;
    private boolean started = false;      // Start pressed (stop() during INIT must not hand off a pose)

    private final List<String> warnings = new ArrayList<>();

    protected RouteAuto(Alliance alliance, Route route) {
        this.alliance = alliance;
        this.route = route;
    }

    // For an auto that picks its route later (SelectAuto). It MUST call
    // useRoute() in init().
    protected RouteAuto() {}

    // Switch to a different alliance/route/wait. Rebuilds the paths and reruns
    // the path check, so the INIT screen always matches what will really run.
    // Call it during init / init_loop only -- never after Start.
    protected void useRoute(Alliance alliance, Route route, double waitSeconds) {
        this.alliance = alliance;
        this.route = route;
        this.waitSeconds = waitSeconds;
        targets.clear();
        paths.clear();
        warnings.clear();
        buildPaths();
        checkPaths();
    }

    // How long (seconds after Start) a YIELD step waits. Only matters for
    // routes that have a YIELD step.
    protected void setYieldUntil(double seconds) {
        yieldUntilSeconds = seconds;
    }

    private void buildPaths() {
        startPose = alliance.map(route.start);
        Pose from = startPose;
        for (Route.Step s : route.steps) {
            Pose to = alliance.map(s.target);
            Path path = new Path(new BezierLine(from, to));
            path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
            targets.add(to);
            paths.add(path);
            from = to;
        }
    }

    // Is the start legal, and would any step hit the hive, a flower, or a wall?
    // Runs during init, so we find out BEFORE pressing Start.
    private void checkPaths() {
        warnings.addAll(PathSafety.checkStart("Start", route.start)); // red version, see checkStart
        for (int i = 0; i < paths.size(); i++) {
            Pose to = targets.get(i);
            String name = String.format("Step %d (to %.0f,%.0f)", i + 1, to.getX(), to.getY());
            warnings.addAll(PathSafety.check(name, paths.get(i)));
        }
    }

    private void autonomousPathUpdate() {
        if (step == -1) {
            // Sitting at the start. After the wait (if any), start the first step.
            if (pathTimer.getElapsedTimeSeconds() > waitSeconds) {
                startStep(0);
            }
            return;
        }
        if (step >= paths.size()) {
            return; // done
        }

        // COLLECT step: turn the collector on once we're close to the point.
        if (route.steps.get(step).action == Route.Action.COLLECT && !collectorOn) {
            Pose here = follower.getPose();
            Pose target = targets.get(step);
            if (Math.hypot(here.getX() - target.getX(), here.getY() - target.getY()) <= COLLECT_RADIUS) {
                collectorOn = true;   // Launcher runs it (controls.collectIn), see loop()
            }
        }

        // isBusy() is true until the robot reaches the end of the step.
        if (!follower.isBusy()) {
            collectorOn = false;

            // LAUNCH / TIP_LAUNCH step: we've arrived, so launch, and stay put until it's done.
            Route.Action action = route.steps.get(step).action;
            if (isLaunch(action)) {
                if (!launching) {
                    launching = true;
                    launchFiring = false;
                    launchTimer.resetTimer();
                    launcher.setFlywheel(true);   // normally already spinning (pre-spun at Start)
                    tipSeen = action != Route.Action.TIP_LAUNCH;   // plain LAUNCH doesn't wait
                    tipFrames = 0;
                    lastTipFrame = 0;
                }
                // cutForPark steps with a park after them: at PARK_LEAVE_BY_SEC,
                // stop whatever we're doing (waiting or firing) and go park.
                boolean mustLeave = route.steps.get(step).cutForPark && step + 1 < paths.size()
                        && runTimer.getElapsedTimeSeconds() >= PARK_LEAVE_BY_SEC;
                if (!tipSeen) {
                    if (mustLeave) {
                        launchNote = "no tip seen -- parking";
                    } else if (tipTagsSeen()) {
                        tipSeen = true;
                        launchTimer.resetTimer();   // aim wait counts from the tip, not arrival
                        return;
                    } else {
                        launchNote = String.format("waiting for tip: %s tags %d / %d frames (leave at %.0f s)",
                                aimCell.displayName, tipFrames, TIP_CONFIRM_FRAMES, PARK_LEAVE_BY_SEC);
                        return;
                    }
                } else if (mustLeave) {
                    launchNote = "out of time -- stopped firing, parking";
                } else if (!launchDone()) {
                    return; // still launching -- check again next loop
                }
                launching = false;
                launchFiring = false;
                if (!hasLaunchAfter(step)) launcher.setFlywheel(false);   // nothing left to shoot
            }

            // YIELD step: we've arrived, so hold still until the yield time.
            if (route.steps.get(step).action == Route.Action.YIELD) {
                yielding = runTimer.getElapsedTimeSeconds() < yieldUntilSeconds;
                if (yielding) {
                    return; // stay out of our partner's way -- check again next loop
                }
            }

            if (step + 1 < paths.size()) {
                startStep(step + 1);
            } else {
                step = paths.size();
                finishedSeconds = runTimer.getElapsedTimeSeconds();
                telemetry.speak("Done"); // spoken by the Driver Station
            }
        }
    }

    /**
     * One loop of a LAUNCH step; true once it's finished. Fire waits for the
     * turret to be on target (or AIM_WAIT_SEC), then keeps firing -- the
     * Launcher feeds one ball per dip, pausing for the wheels to recover.
     */
    private boolean launchDone() {
        double t = launchTimer.getElapsedTimeSeconds();
        int shots = launcher.getShooter().getShotsFired() - shotsAtLaunchStart;

        if (!launchFiring) {
            boolean aimed = aim.isOnTarget() || !Turret.isMeasured();   // no turret = nothing to wait for
            if (aimed || t >= AIM_WAIT_SEC) {
                launchFiring = true;
                fireStartSec = t;
                shotsAtLaunchStart = launcher.getShooter().getShotsFired();
                launchNote = aimed ? "aimed - firing" : "aim timed out - firing anyway";
                // Don't check wasNoBall() yet: it may still be set from the LAST
                // launch. This loop's launcher.update() calls fire(), which clears it.
                return false;
            } else {
                launchNote = String.format("aiming (%.1f s)  %s", t, aim.getStatus());
                return false;
            }
        }

        double firing = t - fireStartSec;
        double stepFireSec = route.steps.get(step).fireSec;
        double maxSec = Double.isNaN(stepFireSec) ? LAUNCH_MAX_SEC : stepFireSec;   // per-route override
        if (LAUNCH_FULL_TIME) {
            // Fixed time: the Launcher re-queues a shot whenever none is pending,
            // so it keeps feeding through "no ball" until the time is up.
            if (firing >= maxSec) {
                launchNote = String.format("done: %.0f s, %d shots counted", maxSec, shots);
                return true;
            }
            launchNote = String.format("firing %.1f / %.0f s, %d counted  (%s)", firing, maxSec,
                    shots, launcher.getShooter().getNote());
            return false;
        }
        if (shots >= LAUNCH_SHOTS) {
            launchNote = String.format("done: %d shots in %.1f s", shots, t);
            return true;
        }
        if (launcher.getShooter().wasNoBall()) {
            launchNote = String.format("done: no ball after %d shots (%.1f s)", shots, t);
            return true;
        }
        if (firing >= maxSec) {
            launchNote = String.format("done: TIMED OUT at %d shots", shots);
            return true;
        }
        launchNote = String.format("firing: %d of %d  (%s)", shots, LAUNCH_SHOTS,
                launcher.getShooter().getNote());
        return false;
    }

    /**
     * Route done: stop aiming, swing the turret to the magnet (HOME_SWITCH_ANGLE,
     * this robot's), then cut its power and leave it there until TeleOp. Aimed at
     * the hive from the park it hangs over the back of the arena (Shane,
     * 2026-11-06). Gives up after STOW_MAX_SEC rather than chase forever.
     */
    private void stowTurret() {
        if (turretStowed) return;   // power already cut; leave it alone
        if (stowTimer == null) {
            stowTimer = new Timer();
            if (turret.isHomed()) {
                turret.setControlMode(Turret.ControlMode.HOLD);
                turret.setTargetAngle(Turret.HOME_SWITCH_ANGLE);
            }
        }
        if (!turret.isHomed() || turret.isOnTarget()
                || stowTimer.getElapsedTimeSeconds() > STOW_MAX_SEC) {
            turret.stop();
            turretStowed = true;
            return;
        }
        turret.update();
    }

    private boolean hasLaunchAfter(int s) {
        for (int i = s + 1; i < route.steps.size(); i++) {
            if (isLaunch(route.steps.get(i).action)) return true;
        }
        return false;
    }

    private static boolean isLaunch(Route.Action a) {
        return a == Route.Action.LAUNCH || a == Route.Action.TIP_LAUNCH;
    }

    private boolean routeLaunches() {
        return route.has(Route.Action.LAUNCH) || route.has(Route.Action.TIP_LAUNCH);
    }

    /**
     * TIP_LAUNCH: has the camera now seen one of OUR side cell's tags (aimCell)
     * in TIP_CONFIRM_FRAMES different frames? Counting frames, not loops: the
     * loop runs faster than the camera and sees each frame several times.
     */
    private boolean tipTagsSeen() {
        long newest = 0;
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection d : camera.getDetections()) {
            if (d.metadata != null && aimCell.hasTag(d.id) && d.frameAcquisitionNanoTime > newest) {
                newest = d.frameAcquisitionNanoTime;
            }
        }
        if (newest != 0 && newest != lastTipFrame) {
            lastTipFrame = newest;
            tipFrames++;
        }
        return tipFrames >= TIP_CONFIRM_FRAMES;
    }

    private void startStep(int newStep) {
        step = newStep;
        follower.followPath(paths.get(step), true); // true = hold position at the end
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        org.firstinspires.ftc.teamcode.RobotProfile.install();   // this robot's turret and camera numbers
        pathTimer = new Timer();
        launchTimer = new Timer();
        runTimer = new Timer();

        RobotHardware hw = new RobotHardware();
        hw.init(hardwareMap, false);   // TurretCamera owns the webcam
        launcher = new Launcher(hw, hardwareMap);
        turret = hw.turret;
        camera = new TurretCamera(hardwareMap);
        deflector = new Deflector(hardwareMap);
        // Every auto, in INIT: down to the hard stop, up to the stow, power off
        // (Shane, 2026-11-08). Wait for HOMED before signaling ready.
        // Homing is NOT started here any more -- locking the menu (A) triggers it,
        // via startHomingIfNeeded(). Starting it in init() meant the settle timers
        // burned through createFollower() and path building before update() ever
        // ran, so it judged the seek before driving the servo once.

        // TURRET HOMES IN INIT TOO, as of 2026-11-10, to stop costing up to
        // HOMING_TIMEOUT_SEC (15 s) of auto -- aiming is gated on isHomed(), so a
        // slow home delays the first shot.
        //
        // >>> NOT CONFIRMED LEGAL. Movement during an auto INIT is fine by
        // >>> G304.H as long as the robot ends motionless, which is what the
        // >>> deflector already relies on. The open question is the SWEEP:
        // >>> homing hunts up to HOMING_SEARCH_MAX_DEG (150 deg) and the camera
        // >>> rides on the turret, so the robot may leave the 18 in. starting
        // >>> cube part-way through even though it ends inside it. A mentor needs
        // >>> to confirm against G304 before this runs at a competition.
        // >>>
        // >>> Revisit when the turret is flipped to face away from the collector
        // >>> (planned rebuild): that is expected to keep the sweep inside the
        // >>> envelope and is the natural moment to settle this.


        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER);
        if (route != null) {
            useRoute(alliance, route, waitSeconds);
        }
    }

    // Runs over and over after INIT is pressed, until Start
    /**
     * Start homing the deflector and the turret, if they need it. Idempotent:
     * safe to call repeatedly, does nothing while homing is already running or
     * already done, and retries after a FAILED attempt so the drive team can just
     * unlock and lock again.
     *
     * Called when the menu is LOCKED, not from init(). Two reasons: the trigger
     * then happens long after construction, so homing settle windows cannot be
     * consumed by follower and path building; and the mechanisms visibly moving
     * tells the drive team the lock took, which is the fix for forgetting to
     * press A.
     */
    protected void startHomingIfNeeded() {
        if (!AUTO_HOME_IN_INIT) return;

        if (deflector.isPresent() && !deflector.isHoming()
                && (!deflector.isZeroed() || deflector.getHomeState() == Deflector.HomeState.FAILED)) {
            deflector.startHoming();
        }
        if (Turret.isMeasured() && !turret.isHomed()
                && turret.getState() != Turret.State.HOMING) {
            turret.startHoming();
        }
    }

    @Override
    public void init_loop() {
        camera.update();
        // false = no hold-loop motion in INIT. Homing is exempt and DOES move here
        // (G304.H is satisfied because it cuts power when it finishes).
        deflector.update(false);
        // Turret homing also only progresses inside update(), same as the
        // deflector. Without this, startHoming() in init() would set the state and
        // nothing would ever apply power -- the SelectAuto bug, again.
        turret.update();
        addRouteTelemetry();
        telemetry.update();
    }

    // Where we start, the wait, and what the path check found.
    protected void addRouteTelemetry() {
        telemetry.addLine(String.format("%s start: (%.1f, %.1f)   wait %.0f sec",
                alliance, startPose.getX(), startPose.getY(), waitSeconds));
        if (warnings.isEmpty()) {
            telemetry.addLine("Path check: OK");
        } else {
            telemetry.addLine("!! PATH CHECK: " + warnings.size() + " problem(s) !!");
            for (String warning : warnings) {
                telemetry.addLine("  " + warning);
            }
        }
        // Launch readiness -- only matters for routes that launch.
        if (route != null && routeLaunches()) {
            // The turret.s ACTUAL state and note. This used to be a canned line that
            // said homing was happening whether or not it was, which is why "it did
            // not move" took three guesses to diagnose.
            if (!Turret.isMeasured()) {
                telemetry.addLine(">>> Turret NOT MEASURED -- it will not home. Turret Range Test.");
            } else {
                telemetry.addData("Turret", "%s%s -- %s", turret.getState(),
                        turret.isHomed() ? " (homed)" : "", turret.getStatusNote());
                if (!turret.isHomed() && turret.getState() != Turret.State.HOMING) {
                    telemetry.addLine("    LOCK the menu (A) to home it. Face it roughly forward first.");
                }
            }
            if (deflector.isPresent()) {
                if (!deflector.isZeroed() && !deflector.isHoming()) {
                    telemetry.addLine("!! Deflector NOT zeroed -- " + deflector.getHomeNote());
                } else {
                    telemetry.addData("Deflector", deflector.getHomeNote());
                }
            }
            if (launcher.isPivotBlocked()) {
                telemetry.addLine("!! Pivot not measured on this robot -- nectar guard can't move it.");
            }
            telemetry.addData("Camera", camera.getNote());
        }
    }

    @Override
    public void start() {
        // Set here, not in init, because SelectAuto can change the start
        // right up until Start is pressed.
        follower.setStartingPose(startPose); // tell the robot where it is at the beginning
        started = true;
        step = -1;
        pathTimer.resetTimer();
        runTimer.resetTimer();

        // Fallback only: INIT normally homes it now. A no-op when already homed,
        // and the retry that covers an INIT too short to finish the sweep.
        if (!turret.isHomed() && Turret.isMeasured()) turret.startHoming();
        aimCell = HiveGeometry.sideCell(alliance, startPose.getY(), null);
        aim = new TurretAim(aimCell.tagIds);
        launcher.start();
        // Still homing at Start? goTo just sets the target; it goes there once homed.
        if (deflector.isZeroed() || deflector.isHoming()) deflector.goTo(Deflector.DEFAULT_SHOT_DEG);
        // Pre-spin, so the first launch spot doesn't wait 1-2 s for spin-up.
        if (routeLaunches()) launcher.setFlywheel(true);
    }

    @Override
    public void loop() {
        follower.update();       // must run every loop or the robot won't move/hold
        camera.update();
        autonomousPathUpdate();  // BEFORE launcher.update() -- see launchDone()

        // Turret: aim the whole way at OUR cell on the side of the field we're
        // on, so it's already on target when we arrive at a launch spot.
        FieldAprilTags.Cell side = HiveGeometry.sideCell(alliance, follower.getPose().getY(), aimCell);
        if (side != aimCell) {
            aimCell = side;
            aim.setTargetTagIds(aimCell.tagIds);
        }
        if (finishedSeconds < 0) {
            aim.update(camera.getDetections(), turret, turret.isHomed());
            turret.update();
        } else {
            stowTurret();
        }

        controls.collectIn = collectorOn;
        // Launch = collector + indexer + flywheels, like RT in TeleOp: the
        // collector helps push balls up (Shane, 2026-11-06). Same at-speed gate
        // and nectar guard as B.
        controls.allForward = launching && launchFiring;
        launcher.update(controls);
        deflector.update(true);
        // Every loop, not just in stop(): if auto is cut off, TeleOp still gets
        // a pose from a moment ago.
        PoseHandoff.save(follower.getPose(), alliance);

        // Auto is 30 seconds -- this shows how much of it the route uses.
        if (finishedSeconds >= 0) {
            telemetry.addLine(String.format("DONE in %.1f sec (of 30)", finishedSeconds));
        } else {
            telemetry.addData("Time (sec)", "%.1f", runTimer.getElapsedTimeSeconds());
        }
        telemetry.addData("Path check problems", warnings.size());
        telemetry.addData("Step", (step + 1) + " of " + paths.size());
        telemetry.addData("Collector", collectorOn ? "ON" : "off");
        if (launching) telemetry.addData("LAUNCH", launchNote);
        telemetry.addData("Flywheel", "%s  L %.0f / R %.0f", launcher.getShooter().getNote(),
                launcher.getShooter().getLeftRpm(), launcher.getShooter().getRightRpm());
        telemetry.addData("Turret", "%s %+.0f deg | %s -> %s", turret.getState(), turret.getAngle(),
                aimCell.displayName, aim.getStatus());
        if (launcher.getGuard().getState() != org.firstinspires.ftc.teamcode.shooting.NectarGuard.State.CLEAR) {
            telemetry.addData("Guard", launcher.getGuard().getNote());
        }
        if (yielding) telemetry.addLine(String.format("YIELDING until %.0f sec", yieldUntilSeconds));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Nothing keeps spinning if Stop is pressed mid-run.
        if (launcher != null) {
            launcher.setFlywheel(false);
            launcher.update(new Launcher.Controls());   // all-false: wheels, indexer, collector off
        }
        if (turret != null) turret.stop();
        if (deflector != null) deflector.stop();
        if (camera != null) camera.close();
        // Where auto ended, for DriveTeleOp's starting pose.
        // Only after Start: stopped during INIT, the pose is just a default.
        if (follower != null && started) PoseHandoff.save(follower.getPose(), alliance);
    }
}
