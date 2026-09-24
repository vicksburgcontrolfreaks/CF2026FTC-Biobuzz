package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * AutoLongShot (No Camera): last season's competition auto (AutoLongShot, 2025),
 * with the camera, shooter, and collector taken out. It ONLY drives the paths.
 * Use it to test Pedro Pathing and the drivetrain -- it is last year's route.
 *
 * What was changed from the 2025 AutoLongShot:
 *   - Alliance is always RED (last year the camera read AprilTag 20 to decide).
 *   - First spike is always spike 2 (last year the camera read the obelisk tag;
 *     spike 2 was also its fallback when no tag was seen). Second spike is spike 1.
 *   - Every "shoot" is a SHOOT_SECONDS pause. Collecting is just driving slowly.
 *   - Poses are last year's FINAL values, copied here on purpose: this repo's
 *     AutonConstants has an older, pre-fix version of them.
 *
 * Route:
 *   start -> long score (pause) -> spike 2 -> spike 2 post -> short score (pause)
 *         -> spike 1 -> spike 1 post -> short score (pause) -> gate release
 */
@Autonomous(name = "AutoLongShot (No Camera)", group = "Autonomous")
public class AutoLongShotNoCam extends OpMode {

    // Red poses from the 2025 repo's final AutonConstants (commit 745b57c)
    private final Pose startPose      = new Pose(88,  9,  Math.toRadians(270));
    private final Pose longScore      = new Pose(88,  14, Math.toRadians(250));
    private final Pose spike2         = new Pose(102, 58, Math.toRadians(0));
    private final Pose spike2Post     = new Pose(124, 58, Math.toRadians(0));
    private final Pose spike1         = new Pose(102, 83, Math.toRadians(0));
    private final Pose spike1Post     = new Pose(124, 83, Math.toRadians(0));
    private final Pose shortScore     = new Pose(89,  80, Math.toRadians(231));
    private final Pose gateRelease    = new Pose(120, 70, Math.toRadians(0));

    private static final double SHOOT_SECONDS = 2.0;          // stands in for a 3-ball shot
    private static final double COLLECTION_MAX_POWER = 0.6;   // same slow speed as last year

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Most paths are built when the step starts, from wherever the robot actually
    // is (follower.getPose()), just like last year's code did.
    private Path line(Pose from, Pose to) {
        Path path = new Path(new BezierLine(from, to));
        path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        return path;
    }

    // Curve back through the spike so the robot doesn't cut through other spikes
    private Path curve(Pose from, Pose via, Pose to) {
        Path path = new Path(new BezierCurve(from, via, to));
        path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        return path;
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Drive to the long shot spot
                follower.followPath(line(startPose, longScore), true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) setPathState(2); // arrived: start the "shot"
                break;
            case 2:
                // "Shooting" the preload, then head slowly to spike 2
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_SECONDS) {
                    follower.setMaxPower(COLLECTION_MAX_POWER);
                    follower.followPath(line(follower.getPose(), spike2), false);
                    setPathState(3);
                }
                break;
            case 3:
                // At spike 2: drive through the balls to the post (still slow)
                if (!follower.isBusy()) {
                    follower.followPath(line(follower.getPose(), spike2Post), false);
                    setPathState(4);
                }
                break;
            case 4:
                // Done "collecting": full speed back to the short shot spot
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(curve(follower.getPose(), spike2, shortScore), true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) setPathState(6);
                break;
            case 6:
                // "Shooting" cycle 1, then slowly to spike 1
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_SECONDS) {
                    follower.setMaxPower(COLLECTION_MAX_POWER);
                    follower.followPath(line(follower.getPose(), spike1), false);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(line(follower.getPose(), spike1Post), false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(curve(follower.getPose(), spike1, shortScore), true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) setPathState(10);
                break;
            case 10:
                // "Shooting" cycle 2, then park at the gate
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_SECONDS) {
                    follower.followPath(line(follower.getPose(), gateRelease), true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) setPathState(-1); // done
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();       // must run every loop or the robot won't move/hold
        autonomousPathUpdate();

        telemetry.addData("Step", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
