package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Red Start Up auto:
 *   1. Start at (59, 8) facing 180 degrees, wait 1 second
 *   2. Drive to (27, 8) facing 180 degrees, wait 1 second
 *   3. Drive 19 more inches to (8, 8), turning from 180 to 0 degrees on the way
 *
 * The robot is controlled by a "state machine": pathState is a number that
 * says which step we're on. loop() runs over and over (many times a second),
 * and each time it checks "is this step done yet?" If yes, it starts the next
 * step and changes pathState.
 */
@Autonomous(name = "Red Start Up", group = "Autonomous")
public class RedStartUp extends OpMode {

    // Where the robot goes. Pose = (x inches, y inches, heading in radians)
    private final Pose startPose  = new Pose(59, 8,   Math.toRadians(180));
    private final Pose secondPose = new Pose(27, 8,   Math.toRadians(180));
    private final Pose finalPose  = new Pose(8,   8,   Math.toRadians(0));

    // How long to wait at each stop
    private static final double WAIT_SECONDS = 1.0;

    private Follower follower;    // Pedro Pathing: drives the robot along paths
    private Timer pathTimer;      // measures time since the current step started
    private int pathState;        // which step we're on

    private Path startToSecond;
    private Path secondToFinal;

    private void buildPaths() {
        // Straight line left along the wall, keep facing 180 the whole way
        startToSecond = new Path(new BezierLine(startPose, secondPose));
        startToSecond.setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading());

        // Straight line 19 more inches along the wall, turning slowly from 180 to 0 the whole way there
        secondToFinal = new Path(new BezierLine(secondPose, finalPose));
        secondToFinal.setLinearHeadingInterpolation(secondPose.getHeading(), finalPose.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Sitting at the start. After 1 second, drive to the second spot.
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(startToSecond, true); // true = hold position at the end
                    setPathState(1);
                }
                break;
            case 1:
                // Driving. isBusy() is true until the robot reaches the end of the path.
                if (!follower.isBusy()) {
                    setPathState(2); // this also restarts the timer for the wait
                }
                break;
            case 2:
                // Waiting at the second spot (the follower keeps holding us there).
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(secondToFinal, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Driving to the final spot. When we arrive, we're done.
                if (!follower.isBusy()) {
                    setPathState(-1); // -1 isn't a case above, so nothing else happens
                }
                break;
        }
    }

    // Move to a new step and restart the step timer
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // tell the robot where it is at the beginning
        buildPaths();
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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
