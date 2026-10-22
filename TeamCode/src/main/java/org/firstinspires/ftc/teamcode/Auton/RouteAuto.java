package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * Drives any Route, for either alliance. Every route auto uses this same
 * code; the route itself comes from Routes.
 *
 * Each Driver Station entry (RedStartUp, BlueStartUp, ...) is a tiny file
 * that picks an alliance and a route, then lets this class do everything.
 * (SelectAuto instead lets the drive team pick them on the gamepad during
 * INIT, by calling useRoute().)
 *
 *   1. Start at the route's start (after WAIT_SECONDS, 0 = go right away)
 *   2. Drive to each step's point, stopping at each one
 *      - COLLECT steps: collector on once within COLLECT_RADIUS of the
 *        point, off when we get there
 *      - LAUNCH steps: on arrival, launch, wait LAUNCH_SECONDS, move on
 *      - YIELD steps: on arrival, hold still until yieldUntilSeconds
 *   3. Say "Done" on the Driver Station, and show how long the run took
 *
 * "abstract" means you can't run this class by itself -- only the tiny files
 * that pick an alliance and a route.
 */
public abstract class RouteAuto extends OpMode {

    private static final double COLLECTOR_POWER = 0.5; // same as DriveTeleOp

    // On a COLLECT step, turn the collector on when the robot gets this close
    // (inches) to the step's point.
    private static final double COLLECT_RADIUS = 22;

    // How long to wait before starting, for the tiny files. 0 = go right away.
    // (SelectAuto picks its own wait on the gamepad.)
    private static final double WAIT_SECONDS = 0;

    // How long a LAUNCH step sits still. A GUESS for now (8 pollen tips the
    // hive) -- the launcher isn't built yet. Time the real one and fix this.
    private static final double LAUNCH_SECONDS = 3;

    // Top speed while testing (1.0 = full speed). Raise it once the robot stops where it should.
    private static final double MAX_POWER = 0.85;

    private Alliance alliance;
    private Route route;
    private double waitSeconds = WAIT_SECONDS;

    private Follower follower;
    private DcMotorEx collector;
    private Timer pathTimer;
    private boolean collectorOn = false;

    private Timer launchTimer;
    private boolean launching = false;

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
                collector.setPower(COLLECTOR_POWER);
                collectorOn = true;
            }
        }

        // isBusy() is true until the robot reaches the end of the step.
        if (!follower.isBusy()) {
            if (collectorOn) {
                collector.setPower(0);
                collectorOn = false;
            }

            // LAUNCH step: we've arrived, so launch, and stay put until it's done.
            if (route.steps.get(step).action == Route.Action.LAUNCH) {
                if (!launching) {
                    launching = true;
                    launchTimer.resetTimer();
                    startLaunch();
                }
                if (launchTimer.getElapsedTimeSeconds() < LAUNCH_SECONDS) {
                    return; // still launching -- check again next loop
                }
                stopLaunch();
                launching = false;
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

    // PLACEHOLDER: the launcher isn't built yet, so these do nothing. The robot
    // just sits for LAUNCH_SECONDS, so timing tests still count the launch.
    // When the launcher works: spin it up / feed pollen in startLaunch(),
    // turn it all off in stopLaunch().
    private void startLaunch() {
    }

    private void stopLaunch() {
    }

    private void startStep(int newStep) {
        step = newStep;
        follower.followPath(paths.get(step), true); // true = hold position at the end
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        launchTimer = new Timer();
        runTimer = new Timer();

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.REVERSE); // same as RobotHardware

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER);
        if (route != null) {
            useRoute(alliance, route, waitSeconds);
        }
    }

    // Runs over and over after INIT is pressed, until Start
    @Override
    public void init_loop() {
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
    }

    @Override
    public void start() {
        // Set here, not in init, because SelectAuto can change the start
        // right up until Start is pressed.
        follower.setStartingPose(startPose); // tell the robot where it is at the beginning
        step = -1;
        pathTimer.resetTimer();
        runTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();       // must run every loop or the robot won't move/hold
        autonomousPathUpdate();

        // Auto is 30 seconds -- this shows how much of it the route uses.
        if (finishedSeconds >= 0) {
            telemetry.addLine(String.format("DONE in %.1f sec (of 30)", finishedSeconds));
        } else {
            telemetry.addData("Time (sec)", "%.1f", runTimer.getElapsedTimeSeconds());
        }
        telemetry.addData("Path check problems", warnings.size());
        telemetry.addData("Step", (step + 1) + " of " + paths.size());
        telemetry.addData("Collector", collectorOn ? "ON" : "off");
        if (launching) telemetry.addLine("LAUNCHING (placeholder -- nothing moves)");
        if (yielding) telemetry.addLine(String.format("YIELDING until %.0f sec", yieldUntilSeconds));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Make sure the collector doesn't keep spinning if Stop is pressed mid-run
        if (collector != null) collector.setPower(0);
    }
}
