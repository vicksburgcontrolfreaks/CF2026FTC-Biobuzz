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
import org.firstinspires.ftc.teamcode.subsystems.TurretDirector;
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
 *      - LAUNCH steps: on arrival, SPINUP_SEC of flywheel only, then
 *        collector + indexer + flywheels for the step's fire time (or
 *        LAUNCH_MAX_SEC), move on. Pure clock -- no aim wait, no speed gate.
 *        The flywheel pre-spins from Start; the turret aims by camera the
 *        whole way at our cell on our side of the field.
 *      - YIELD steps: on arrival, hold still until yieldUntilSeconds
 *      - a step with a timeout (Route.timeout) that can't get there in time is
 *        abandoned, and the next step picks up from wherever the robot is
 *   3. If the route marked its park steps (Route.beginPark), PARK_LEAVE_BY_SEC
 *      is a hard deadline: at that point the robot quits whatever it is doing
 *      and drives the park, so a stuck step can't cost the park too
 *   4. Say "Done" on the Driver Station, and show how long the run took
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

    // ---- Launching: PURE CLOCK (Shane, 2026-11-14) ----------------------------
    // Every launch step in every route: SPINUP_SEC with the flywheels on and
    // nothing feeding, then collector + indexer + flywheels for the step's fire
    // time (LAUNCH_MAX_SEC if the route didn't give one). That's all.
    //
    // NO aim wait: the turret aims during INIT and the whole way there, so it is
    // already on target. NO at-speed gate: Launcher.useUngatedFeed() turns it off
    // for autos (TeleOp keeps it). NO shot counting.
    //
    // WHY: Garden Shoot 2X fired nothing on 2026-11-14. The 4 s fire window
    // started when the AIM finished -- about t = 0 now that INIT aims -- while
    // the indexer was still waiting for both wheels to settle within 50 RPM.
    // Spin-up ate the window and the Driver Station said "firing" throughout.
    //
    // SPIN-UP IS SKIPPED when the wheels have ALREADY been on for SPINUP_SEC
    // (Shane, 2026-11-14): the clock is "flywheel on for 2 s", not "2 s per
    // launch". So a launch at the start waits 2 s, but Garden Shoot 2X's second
    // launch (wheels on since Start) and any launch reached by a drive longer
    // than 2 s feed immediately. See flywheel() and flywheelOnSinceSec.
    // 2 -> 3 -> 4 (Shane, 2026-11-14 evening): 6494 put the first two auto shots
    // low and the last two in -- wheels not up to speed at 2 s.
    public static double SPINUP_SEC = 4.0;
    /** runTimer time the flywheel was last switched on; NaN = off. Set only by flywheel(). */
    private double flywheelOnSinceSec = Double.NaN;
    /** Seconds of feeding (collector + indexer + flywheels), counted after SPINUP_SEC. */
    public static double LAUNCH_MAX_SEC = 15.0;   // 8 -> 12 -> 15: still short on shots (Shane, 2026-11-06)

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

    /**
     * Aim the turret at the hive during INIT, once the route is locked in and the
     * turret has homed. See aimDuringInit() for what it does and what it means
     * for G304. false = the old behaviour, turret parked at home until Start.
     */
    public static boolean AIM_IN_INIT = true;

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
    /**
     * Seconds to aim after the tip is confirmed, before feeding (Shane,
     * 2026-11-14, Far Wall Wait and Shoot). Every tip launch, so Far Wall Tip
     * Shoot + Park gets it too.
     */
    public static double TIP_AIM_SEC = 2.0;

    // ---- HIVE CHECK (Garden Shoot 2X Tip Check, Shane 2026-11-15) ------------
    /**
     * LAUNCH_HERE_IF_UP: how long to LOOK for our cell's tags on arrival before
     * deciding the hive has tipped and skipping the shot. Seeing them
     * (TIP_CONFIRM_FRAMES new frames) ends the look early and fires at once, so a
     * normal run only spends the ~0.2 s those frames take. Generous on purpose:
     * the robot has just braked and the turret may still be settling, and a false
     * "tipped" costs the whole second volley.
     */
    public static double HIVE_CHECK_SEC = 1.5;

    // 1.0 = full speed (Shane, 2026-11-06). Was 0.5 while 6494's odometry drifts
    // ~1" per round trip (docs/TODO.md item 0) -- at full speed that drift and
    // braking overshoot both grow, so retest every route on both robots.
    // Back to 1.0 after Garden Shoot + Park ran clean at 0.5 (Shane, 2026-11-06).
    private static final double MAX_POWER = 1.0;

    private Alliance alliance;
    private Route route;
    private double waitSeconds = WAIT_SECONDS;

    /**
     * A redrawn path shorter than this (inches) is treated as no path at all --
     * the follower doesn't cope with zero-length paths. See rebuildPath().
     */
    private static final double MIN_PATH_IN = 1.0;

    private Follower follower;
    private Timer pathTimer;
    /** Why we last gave up on a step, for the Driver Station. "" = never did. */
    private String abandonNote = "";
    private boolean collectorOn = false;
    /** A step is holding on arrival for its dwellSec. See autonomousPathUpdate. */
    private boolean dwelling = false;
    private Timer dwellTimer;

    // Same pieces as DriveTeleOp, so the nectar guard and at-speed gate come along.
    private Launcher launcher;
    private final Launcher.Controls controls = new Launcher.Controls();
    private Turret turret;
    private TurretCamera camera;
    // Camera-only unless USE_ODOMETRY_AIM is on -- tip routes force it, see applyAimSource().
    private TurretDirector aim;
    private FieldAprilTags.Cell aimCell;
    private Deflector deflector;

    private Timer launchTimer;
    private boolean launching = false;
    private boolean launchFiring = false;   // aim wait over, shots going
    private double fireStartSec = 0;         // launchTimer time when firing started
    private boolean tipSeen = false;         // TIP_LAUNCH: our cell's tags confirmed
    private int tipFrames = 0;               // camera frames with our cell's tags in them
    private long lastTipFrame = 0;           // so one frame seen over several loops counts once
    // LAUNCH_HERE_IF_UP: still looking for our cell's tags before deciding.
    private boolean hiveCheckPending = false;
    /** What the hive check decided, for the Driver Station. "" = hasn't run. */
    private String hiveCheckNote = "";
    /**
     * Every hive cell the camera has seen a tag of, since INIT. The hive check's
     * safeguard: "no tags now" only means "tipped" if the camera has PROVED it
     * can see that cell earlier. Never cleared -- a sighting in INIT counts.
     */
    private final java.util.EnumSet<FieldAprilTags.Cell> cellsSeen =
            java.util.EnumSet.noneOf(FieldAprilTags.Cell.class);

    // End of route: turret to the magnet, then off until TeleOp (see stowTurret).
    public static double STOW_MAX_SEC = 3.0;
    private Timer stowTimer = null;
    private boolean turretStowed = false;
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
    /** The route is locked in (A). Set by startHomingIfNeeded(); see aimDuringInit(). */
    private boolean menuLocked = false;

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
        applyAimSource();
    }

    /** RobotProfile's USE_ODOMETRY_AIM for this robot, saved before a route overrides it. */
    private boolean profileOdometryAim = false;

    /**
     * TIP ROUTES AIM BY ODOMETRY (Shane, 2026-11-14). Before the partner tips the
     * hive our cell's tags are out of sight BY DEFINITION, so camera-only aiming
     * just sits searching -- which is what Far Wall Wait and Shoot did on its
     * first run. The pose is all there is, so these routes aim from it (with the
     * camera trim taking over once the tags appear). Every other route keeps the
     * robot's default from RobotProfile; switching routes in the menu restores it.
     *
     * Needs the robot's turret geometry measured (TurretGeometry.isConfigured);
     * without it TurretDirector falls back to the camera, and the INIT screen says
     * so loudly (see addRouteTelemetry).
     */
    private void applyAimSource() {
        TurretDirector.USE_ODOMETRY_AIM = routeWaitsForTip() || profileOdometryAim;
    }

    private boolean routeWaitsForTip() {
        return route != null && (route.has(Route.Action.TIP_LAUNCH)
                || route.has(Route.Action.TIP_LAUNCH_HERE));
    }

    // How long (seconds after Start) a YIELD step waits. Only matters for
    // routes that have a YIELD step.
    protected void setYieldUntil(double seconds) {
        yieldUntilSeconds = seconds;
    }

    /**
     * 6494 PARKS TOO FAR FROM OUR WALL (Shane, 2026-11-15, comp day): on both
     * park spots it ends up further toward the hive than the 4 in. wall gap the
     * spots are drawn for -- most likely its strafe-pod drift (docs/TODO.md item
     * 0), not the spots themselves. So on 6494 ONLY, the FINAL park point is
     * moved this many inches toward our alliance wall (commanded gap 4 -> 3 in.).
     * 3 -> 1 (Shane, 2026-11-15): 2 in. added back, commanded gap now 3 in.
     * 5618 parks where it always did. The earlier park legs are untouched -- they
     * are what keep us off the hive and the red-wall flower.
     */
    public static double PARK_WALL_SHIFT_6494 = 1.0;   // 3 -> 1, Shane 2026-11-15
    /** This robot's shift, set once in init(). 0 on 5618 or an unknown robot. */
    private double parkWallShift = 0;

    private static double parkWallShiftForThisRobot() {
        try {
            return org.firstinspires.ftc.teamcode.WhichRobot.get()
                    == org.firstinspires.ftc.teamcode.WhichRobot.Robot.R6494 ? PARK_WALL_SHIFT_6494 : 0;
        } catch (IllegalStateException e) {
            return 0;   // can't tell which robot: park where the spots say
        }
    }

    private void buildPaths() {
        startPose = alliance.map(route.start);
        Pose from = startPose;
        int last = route.steps.size() - 1;
        for (int i = 0; i <= last; i++) {
            Route.Step s = route.steps.get(i);
            // LAUNCH_HERE goes nowhere: NULL path, target = where we already are.
            // The lists stay index-parallel with route.steps, which everything
            // here indexes by, and a null path counts as arrived (see
            // autonomousPathUpdate). A zero-length BezierLine would be the
            // obvious alternative and is exactly what we are avoiding.
            if (s.target == null) {   // LAUNCH_HERE, TIP_LAUNCH_HERE
                targets.add(from);
                paths.add(null);
                continue;
            }
            // The park point is the LAST step of a route that marked a park.
            // Shift it in RED coordinates, before the alliance mirror, so blue
            // moves toward the blue wall the same way.
            Pose red = s.target;
            if (route.parkFrom >= 0 && i == last && parkWallShift != 0) {
                red = Waypoints.towardRedWall(red, parkWallShift);
            }
            Pose to = alliance.map(red);
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
            if (paths.get(i) == null) continue;   // LAUNCH_HERE: no path to check
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

        // PARK DEADLINE (Shane, 2026-11-14) -- the backstop for "get to the park
        // no matter what". If the route marked where its park steps start
        // (Route.beginPark) and we are still short of them at PARK_LEAVE_BY_SEC,
        // drop whatever we are doing -- driving, collecting, dwelling, firing --
        // and go park. This is checked FIRST, before anything else in the step,
        // so nothing can hold the robot past the deadline.
        //
        // It covers the launch steps' own cutForPark and then some: cutForPark
        // only stops FIRING and moves to the next step, which could be more
        // collecting, while this jumps straight to the park. Both trigger at the
        // same second; this one gets there first and wins.
        if (route.parkFrom >= 0 && step < route.parkFrom
                && runTimer.getElapsedTimeSeconds() >= PARK_LEAVE_BY_SEC) {
            jumpTo(route.parkFrom,
                    String.format("park deadline (%.0f s)", PARK_LEAVE_BY_SEC));
            return;
        }

        // COLLECT step: turn the collector on once we're close to the point.
        if (route.steps.get(step).action == Route.Action.COLLECT && !collectorOn) {
            Pose here = follower.getPose();
            Pose target = targets.get(step);
            if (Math.hypot(here.getX() - target.getX(), here.getY() - target.getY()) <= COLLECT_RADIUS) {
                collectorOn = true;   // Launcher runs it (controls.collectIn), see loop()
            }
        }

        // isBusy() is true until the robot reaches the end of the step. A
        // LAUNCH_HERE step has no path, so it is arrived from the first loop.
        boolean arrived = paths.get(step) == null || !follower.isBusy();

        // STEP TIMEOUT (Shane, 2026-11-14): still driving and out of time for
        // this leg, so stop trying to reach the point and take the next step from
        // wherever we actually are. See Route.Step.timeoutSec for why -- short
        // version: isBusy() stays true forever against a ball, and a step that
        // never ends takes the rest of the route with it.
        //
        // Only while !arrived: once the robot is there, the dwell and the launch
        // own the clock, and timing them here would cut shots short.
        double limit = route.steps.get(step).timeoutSec;
        if (!arrived && !Double.isNaN(limit) && pathTimer.getElapsedTimeSeconds() >= limit) {
            jumpTo(step + 1, String.format("step %d timed out (%.0f s)", step + 1, limit));
            return;
        }

        if (arrived) {
            // DWELL: hold on arrival with whatever the step was doing still
            // running, BEFORE the collector is switched off below. For a COLLECT
            // step that is the point -- the balls that reached the wall with us
            // still have to be swept up.
            Route.Step current = route.steps.get(step);
            if (!Double.isNaN(current.dwellSec) && current.dwellSec > 0) {
                if (!dwelling) {
                    dwelling = true;
                    dwellTimer.resetTimer();
                }
                if (dwellTimer.getElapsedTimeSeconds() < current.dwellSec) {
                    return;   // still dwelling -- collector (if any) stays on
                }
            }
            collectorOn = false;

            // LAUNCH / TIP_LAUNCH step: we've arrived, so launch, and stay put until it's done.
            Route.Action action = route.steps.get(step).action;
            if (isLaunch(action)) {
                if (!launching) {
                    launching = true;
                    launchFiring = false;
                    launchTimer.resetTimer();
                    flywheel(true);    // normally already spinning (pre-spun at Start)
                    tipSeen = !isTip(action);   // plain LAUNCH doesn't wait
                    tipFrames = 0;
                    lastTipFrame = 0;
                    hiveCheckPending = action == Route.Action.LAUNCH_HERE_IF_UP;
                    // ONLY FRAMES FROM NOW ON COUNT: start the frame counter at the
                    // newest frame already in hand, so a sighting from the drive back
                    // (before the hive maybe tipped) can't vote. Compares camera
                    // timestamps with each other only, never with our own clock.
                    if (hiveCheckPending) lastTipFrame = newestCellFrame();
                }
                // cutForPark steps with a park after them: at PARK_LEAVE_BY_SEC,
                // stop whatever we're doing (waiting or firing) and go park.
                boolean mustLeave = route.steps.get(step).cutForPark && step + 1 < paths.size()
                        && runTimer.getElapsedTimeSeconds() >= PARK_LEAVE_BY_SEC;

                // HIVE CHECK (LAUNCH_HERE_IF_UP, Shane 2026-11-15): is our cell
                // still up? Decide, then either fall into the normal launch below
                // (next loop) or skip the shot.
                boolean skipShot = false;
                if (hiveCheckPending) {
                    String cell = aimCell.displayName;
                    double looked = launchTimer.getElapsedTimeSeconds();
                    if (!cellsSeen.contains(aimCell)) {
                        // SAFEGUARD: never saw this cell all match -- a dead or
                        // blocked camera looks exactly like a tip. Can't judge: shoot.
                        hiveCheckPending = false;
                        hiveCheckNote = "camera never saw " + cell + " -- shooting anyway";
                        launchTimer.resetTimer();
                        return;
                    } else if (tipTagsSeen()) {
                        hiveCheckPending = false;
                        hiveCheckNote = String.format("%s tags seen in %.1f s -- not tipped, shooting",
                                cell, looked);
                        launchTimer.resetTimer();
                        return;
                    } else if (looked >= HIVE_CHECK_SEC) {
                        hiveCheckPending = false;
                        hiveCheckNote = String.format("no %s tags in %.1f s -- TIPPED, skipped shot",
                                cell, HIVE_CHECK_SEC);
                        skipShot = true;
                    } else {
                        launchNote = String.format("checking hive: %s tags %d / %d frames (%.1f / %.1f s)",
                                cell, tipFrames, TIP_CONFIRM_FRAMES, looked, HIVE_CHECK_SEC);
                        return;
                    }
                }

                if (skipShot) {
                    launchNote = hiveCheckNote;   // fall through: on to the next step
                } else if (!tipSeen) {
                    if (mustLeave) {
                        launchNote = "no tip seen -- parking";
                    } else if (tipTagsSeen()) {
                        tipSeen = true;
                        launchTimer.resetTimer();   // spin-up counts from the tip, not arrival
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
                if (!hasLaunchAfter(step)) flywheel(false);   // nothing left to shoot
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
                finish();
            }
        }
    }

    private void finish() {
        step = paths.size();
        finishedSeconds = runTimer.getElapsedTimeSeconds();
        telemetry.speak("Done"); // spoken by the Driver Station
    }

    /**
     * Give up on the step we're on and run step `to` instead, starting from
     * WHERE THE ROBOT ACTUALLY IS.
     *
     * That last part is the whole job. buildPaths() draws every leg at INIT from
     * the PREVIOUS leg's target, so step `to`'s path starts at a point we just
     * decided we are never going to reach. Following it from here would have the
     * robot cut across to that line from the side -- an unplanned diagonal, and
     * the 2026-11-14 hive strike is what unplanned diagonals cost around here.
     * So the path gets redrawn from the live pose.
     *
     * The redrawn line was never path-checked at INIT, so it is checked now and
     * anything it finds is added to `warnings` -- the Driver Station's "Path
     * check problems" count goes up, and the detail is there to read afterward.
     * It is a report, not a veto: there is nothing better to do with the time
     * left than drive it.
     *
     * The robot is probably still jammed on whatever stopped it. Nothing here can
     * unjam it -- what unjams it is the next leg pulling AWAY. In Garden Shoot 2X
     * that happens to be true for both legs that can time out (the sweep's
     * escape is back toward blue, the return's is back toward red).
     */
    private void jumpTo(int to, String why) {
        Pose here = follower.getPose();
        abandonNote = String.format("%s -- jumped to step %d from (%.0f, %.0f)",
                why, to + 1, here.getX(), here.getY());
        collectorOn = false;
        dwelling = false;
        launching = false;
        launchFiring = false;
        hiveCheckPending = false;
        if (to >= paths.size()) {   // nothing left to jump to: that was the route
            flywheel(false);
            finish();
            return;
        }
        // Nothing left to shoot from `to` onward? Stop the wheels. (hasLaunchAfter
        // looks at the steps AFTER its argument, so to - 1 means "to onward".)
        if (!hasLaunchAfter(to - 1)) flywheel(false);
        rebuildPath(to, here);
        startStep(to);
    }

    /** Redraw step `index`'s path so it starts at `here`. See jumpTo(). */
    private void rebuildPath(int index, Pose here) {
        if (paths.get(index) == null) return;   // LAUNCH_HERE: no path to redraw
        Pose to = targets.get(index);
        if (Math.hypot(to.getX() - here.getX(), to.getY() - here.getY()) < MIN_PATH_IN) {
            // We are already on the point. A path this short is a zero-length
            // path as far as the follower is concerned (see Action.LAUNCH_HERE),
            // so give the step NO path: it counts as arrived immediately and
            // still does whatever it was going to do there.
            paths.set(index, null);
            targets.set(index, here);
            return;
        }
        Path rebuilt = new Path(new BezierLine(here, to));
        rebuilt.setLinearHeadingInterpolation(here.getHeading(), to.getHeading());
        paths.set(index, rebuilt);
        warnings.addAll(PathSafety.check(
                String.format("Rebuilt step %d (to %.0f,%.0f)", index + 1, to.getX(), to.getY()),
                rebuilt));
    }

    /**
     * One loop of a LAUNCH step; true once it's finished. Pure clock: SPINUP_SEC
     * of flywheel only, then feed for the step's fire time. See SPINUP_SEC.
     */
    private boolean launchDone() {
        double t = launchTimer.getElapsedTimeSeconds();

        if (!launchFiring) {
            double spunSec = Double.isNaN(flywheelOnSinceSec)
                    ? 0 : runTimer.getElapsedTimeSeconds() - flywheelOnSinceSec;
            boolean spunUp = t >= SPINUP_SEC || spunSec >= SPINUP_SEC;   // already spun up = no wait
            // TIP launches also give the turret TIP_AIM_SEC after the tip: the cell
            // just swung up, so the camera trim only starts converging now.
            // launchTimer was reset when the tip was confirmed, so t counts from it.
            boolean aimed = !isTip(route.steps.get(step).action) || t >= TIP_AIM_SEC;
            if (!aimed) {
                launchNote = String.format("tip seen - aiming %.1f / %.0f s  %s", t, TIP_AIM_SEC, aim.getStatus());
                return false;
            }
            if (spunUp) {
                launchFiring = true;   // loop() turns this into allForward
                fireStartSec = t;
                launchNote = "spun up - feeding";
            } else {
                launchNote = String.format("spinning up %.1f / %.0f s", t, SPINUP_SEC);
            }
            return false;
        }

        double firing = t - fireStartSec;
        double stepFireSec = route.steps.get(step).fireSec;
        double maxSec = Double.isNaN(stepFireSec) ? LAUNCH_MAX_SEC : stepFireSec;   // per-route override
        if (firing >= maxSec) {
            launchNote = String.format("done: fed %.0f s", maxSec);
            return true;
        }
        launchNote = String.format("feeding %.1f / %.0f s  (indexer %s)", firing, maxSec,
                launcher.isIndexerForward() ? "ON" : "OFF");
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

    /**
     * Flywheel on/off for the route. ALWAYS use this, never launcher.setFlywheel()
     * directly (stop() excepted), or the spin-up skip in launchDone() will think
     * the wheels have been on longer than they have.
     */
    private void flywheel(boolean on) {
        if (on && Double.isNaN(flywheelOnSinceSec)) {
            flywheelOnSinceSec = runTimer.getElapsedTimeSeconds();
        } else if (!on) {
            flywheelOnSinceSec = Double.NaN;
        }
        launcher.setFlywheel(on);
    }

    private boolean hasLaunchAfter(int s) {
        for (int i = s + 1; i < route.steps.size(); i++) {
            if (isLaunch(route.steps.get(i).action)) return true;
        }
        return false;
    }

    private static boolean isLaunch(Route.Action a) {
        return a == Route.Action.LAUNCH || isTip(a)
                || a == Route.Action.LAUNCH_HERE || a == Route.Action.LAUNCH_HERE_IF_UP;
    }

    /** Waits for the partner to tip the hive before it fires. */
    private static boolean isTip(Route.Action a) {
        return a == Route.Action.TIP_LAUNCH || a == Route.Action.TIP_LAUNCH_HERE;
    }

    private boolean routeLaunches() {
        return route.has(Route.Action.LAUNCH) || route.has(Route.Action.TIP_LAUNCH)
                || route.has(Route.Action.LAUNCH_HERE) || route.has(Route.Action.TIP_LAUNCH_HERE)
                || route.has(Route.Action.LAUNCH_HERE_IF_UP);
    }

    /** Camera timestamp of the newest frame in hand with one of aimCell's tags; 0 = none. */
    private long newestCellFrame() {
        long newest = 0;
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection d : camera.getDetections()) {
            if (d.metadata != null && aimCell.hasTag(d.id) && d.frameAcquisitionNanoTime > newest) {
                newest = d.frameAcquisitionNanoTime;
            }
        }
        return newest;
    }

    /**
     * Remember every hive cell in view right now (see cellsSeen). Called every
     * loop in INIT and the match, right after camera.update(). Returns whether
     * `cell` is in view in THIS frame, for the INIT readout.
     */
    private boolean noteCellsSeen(FieldAprilTags.Cell cell) {
        boolean inView = false;
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection d : camera.getDetections()) {
            if (d.metadata == null) continue;
            FieldAprilTags.Cell c = FieldAprilTags.Cell.forTag(d.id);
            if (c == null) continue;
            cellsSeen.add(c);
            if (c == cell) inView = true;
        }
        return inView;
    }

    /** The cell the hive check will look at: aimCell once aiming, else our start-side cell. */
    private FieldAprilTags.Cell checkCell() {
        return aimCell != null ? aimCell : HiveGeometry.sideCell(alliance, startPose.getY(), null);
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
        dwelling = false;
        // PER-STEP SPEED. Set every step, not just the slow ones, or a slow leg
        // would quietly stay slow for the rest of the route.
        Route.Step s = route.steps.get(step);
        follower.setMaxPower(Double.isNaN(s.speed) ? MAX_POWER : s.speed);
        if (paths.get(step) != null) {
            follower.followPath(paths.get(step), true); // true = hold position at the end
        }
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        org.firstinspires.ftc.teamcode.RobotProfile.install();   // this robot's turret and camera numbers
        profileOdometryAim = TurretDirector.USE_ODOMETRY_AIM;   // this robot's default, before any route overrides it
        pathTimer = new Timer();
        launchTimer = new Timer();
        dwellTimer = new Timer();
        runTimer = new Timer();

        RobotHardware hw = new RobotHardware();
        hw.init(hardwareMap, false);   // TurretCamera owns the webcam
        launcher = new Launcher(hw, hardwareMap);
        launcher.useAutoRpm();   // this robot's AUTO speed (5618: 1250, TeleOp is 1225)
        launcher.useUngatedFeed();   // autos run on the clock, see SPINUP_SEC
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
        parkWallShift = parkWallShiftForThisRobot();   // before any useRoute() builds paths
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
        // SelectAuto calls this on the unlocked -> locked transition and nowhere
        // else, so this is also where we learn the route is locked in. Set before
        // the AUTO_HOME_IN_INIT early return: aiming in INIT is a separate switch.
        menuLocked = true;
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

    /**
     * Point the turret at OUR cell for wherever the route starts. Safe to call
     * repeatedly, and it KEEPS an existing director rather than replacing it, so
     * a trim that converged during INIT is not thrown away at Start.
     */
    private void aimAtStartCell() {
        FieldAprilTags.Cell side = HiveGeometry.sideCell(alliance, startPose.getY(), aimCell);
        if (aim == null) {
            aimCell = side;
            aim = new TurretDirector(aimCell.tagIds);
        } else if (side != aimCell) {
            aimCell = side;
            aim.setTargetTagIds(aimCell.tagIds);
        }
    }

    /**
     * AIM DURING INIT, once the route is locked and the turret has homed
     * (Shane, 2026-11-14): "as soon as the robot starts we are going to be
     * shooting at the hive, so it is imperative that we're aimed before the
     * match start." Every route, not just the shooting ones.
     *
     * THE POSE IS startPose, NOT follower.getPose(). setStartingPose() does not
     * run until start(), so during INIT the follower still believes it is at the
     * origin -- aiming from that would point the turret at nothing. We do know
     * where the robot is: the route says so, and the drive team has set it there.
     *
     * The camera is already running in INIT, so if it can see the cell's tags the
     * trim converges before Start too, and aimAtStartCell() hands that trim to
     * the match instead of starting over.
     *
     * TWO THINGS THE DRIVE TEAM SHOULD KNOW:
     *   - The turret now MOVES on lock (A) and then HOLDS under power until
     *     Start. Previously INIT motion was homing only, which cut power when it
     *     finished. The robot is still motionless when it ends, which is what
     *     G304.H asks, and the deflector already relies on that reading.
     *   - It makes the 18 in. starting-cube question smaller rather than bigger:
     *     the aim is a move to a fixed angle, while homing already sweeps up to
     *     HOMING_SEARCH_MAX. And since the 2026-11-14 remount the camera sits
     *     1.25 in. from the turret axis instead of 4.5, so there is much less
     *     hanging off the turret to swing out. Still wants a mentor's read
     *     against G304 before a competition -- see the note in init().
     */
    private void aimDuringInit() {
        if (!AIM_IN_INIT || !menuLocked) return;
        if (!Turret.isMeasured() || !turret.isHomed()) return;   // nothing to aim yet
        aimAtStartCell();
        double[] opening = HiveGeometry.aimPointFor(aimCell);
        aim.update(camera.getDetections(), turret, startPose, opening[0], opening[1],
                true, getRuntime());
    }

    @Override
    public void init_loop() {
        camera.update();
        checkCellInView = noteCellsSeen(checkCell());
        // false = no hold-loop motion in INIT. Homing is exempt and DOES move here
        // (G304.H is satisfied because it cuts power when it finishes).
        deflector.update(false);
        // Once homed, point at the hive and HOLD it, so the first shot does not
        // spend the start of the match turning. Sets a target only; the single
        // turret.update() below is still the one place the turret moves.
        aimDuringInit();
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
        if (route != null && route.parkFrom >= 0 && parkWallShift != 0) {
            telemetry.addLine(String.format("Park: %.0f in. closer to our wall (this robot only)", parkWallShift));
        }
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
                    // Wording is Jade's, 2026-11-14: one phrase for this on every
                    // screen, on both robots, now that turret 0 faces the rear.
                    telemetry.addLine("    LOCK the menu (A) to home it. Point it roughly BACKWARD first.");
                }
                // AIMED BEFORE START, or say why not. The whole point of aiming in
                // INIT is that the drive team can SEE it is pointed at the hive
                // before the match starts, so this is a readout, not a comment.
                if (AIM_IN_INIT && turret.isHomed()) {
                    if (aim == null) {
                        telemetry.addLine(">>> NOT AIMED YET: lock the menu (A).");
                    } else {
                        telemetry.addData("Aim (INIT)", "%s at %s -- %s",
                                aim.isOnTarget() ? "ON TARGET" : "aiming",
                                aimCell.displayName, aim.getStatus());
                        telemetry.addData("Aim by", "%s%s", aim.getSource(),
                                aim.hasCameraFix()
                                        ? String.format("  |  trim %+.1f deg", aim.getTrimDeg()) : "");
                    }
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
            if (routeWaitsForTip()
                    && !org.firstinspires.ftc.teamcode.geometry.TurretGeometry.forRobot().isConfigured()) {
                telemetry.addLine(">>> TURRET GEOMETRY NOT MEASURED on this robot -- it CANNOT aim");
                telemetry.addLine("    by odometry, so it will only search for tags until the tip.");
            }
            telemetry.addData("Camera", camera.getNote());
            // HIVE CHECK, LIVE (Garden Shoot 2X Tip Check). Tip the hive by hand
            // in INIT and watch this flip: that is the test that the camera really
            // loses the tags when the cell goes down, before risking a match on it.
            if (route.has(Route.Action.LAUNCH_HERE_IF_UP)) {
                FieldAprilTags.Cell cell = checkCell();
                telemetry.addData("Hive check", "%s tags %s now  |  seen since INIT: %s",
                        cell.displayName, checkCellInView ? "SEEN" : "NOT seen",
                        cellsSeen.contains(cell) ? "YES" : "NO -- check would shoot anyway");
            }
        }
    }

    /** INIT readout: is the hive check's cell in view this frame? Set in init_loop(). */
    private boolean checkCellInView = false;

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
        // KEEPS WHATEVER INIT BUILT. aimAtStartCell() is a no-op if INIT already
        // made the director and picked the cell, so a camera trim that converged
        // during INIT survives into the match instead of being thrown away by a
        // fresh TurretDirector at Start. See aimDuringInit().
        aimAtStartCell();
        launcher.start();
        // Still homing at Start? goTo just sets the target; it goes there once homed.
        if (deflector.isZeroed() || deflector.isHoming()) deflector.goTo(Deflector.DEFAULT_SHOT_DEG);
        // Pre-spin, so the first launch spot doesn't wait 1-2 s for spin-up.
        if (routeLaunches()) flywheel(true);
    }

    @Override
    public void loop() {
        follower.update();       // must run every loop or the robot won't move/hold
        camera.update();
        noteCellsSeen(aimCell);  // the hive check's "camera has seen it" safeguard
        autonomousPathUpdate();  // BEFORE launcher.update() -- see launchDone()

        // Turret: aim the whole way at OUR cell on the side of the field we're
        // on, so it's already on target when we arrive at a launch spot.
        FieldAprilTags.Cell side = HiveGeometry.sideCell(alliance, follower.getPose().getY(), aimCell);
        if (side != aimCell) {
            aimCell = side;
            aim.setTargetTagIds(aimCell.tagIds);
        }
        if (finishedSeconds < 0) {
            // Auto always has a pose, so odometry aiming can reach a cell the camera
            // cannot see yet -- the turret is already pointed when the shot comes up.
            double[] opening = HiveGeometry.aimPointFor(aimCell);
            aim.update(camera.getDetections(), turret, follower.getPose(),
                    opening[0], opening[1], turret.isHomed(), runTimer.getElapsedTimeSeconds());
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
        // STEP TIME is how you tune the timeouts: watch a good run, see what each
        // leg really takes, then set the limit comfortably above it. Shows the
        // limit too when the step has one.
        if (step >= 0 && step < paths.size()) {
            double limit = route.steps.get(step).timeoutSec;
            telemetry.addData("Step time", Double.isNaN(limit)
                    ? String.format("%.1f s", pathTimer.getElapsedTimeSeconds())
                    : String.format("%.1f / %.0f s", pathTimer.getElapsedTimeSeconds(), limit));
        }
        if (!abandonNote.isEmpty()) telemetry.addData("GAVE UP", abandonNote);
        telemetry.addData("Collector", collectorOn ? "ON" : "off");
        if (launching) telemetry.addData("LAUNCH", launchNote);
        if (!hiveCheckNote.isEmpty()) telemetry.addData("Hive check", hiveCheckNote);   // stays up after
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
