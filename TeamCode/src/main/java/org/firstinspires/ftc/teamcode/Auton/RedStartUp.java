package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Red Start Up auto:
 *   1. Start at (59, 8) facing 180 degrees, go right away (no wait)
 *   2. Drive straight along the wall to (8, 8), still facing 180 degrees
 *      - when the robot passes x = 31, turn the collector on
 *   3. When it arrives, turn the collector off
 *   4. Drive around the hive, stopping at each corner, still facing 180:
 *      (33, 59) -> (33, 111) -> (59, 111) -> (59, 133)
 *      The robot is ~18 in. wide, so y = 111 keeps it about 4 in. above the hive.
 *   5. Say "Done" on the Driver Station
 *
 * The robot is controlled by a "state machine": pathState is a number that
 * says which step we're on. loop() runs over and over (many times a second),
 * and each time it checks "is this step done yet?" If yes, it starts the next
 * step and changes pathState.
 */
@Autonomous(name = "Red Start Up", group = "Autonomous")
public class RedStartUp extends OpMode {

    // Where the robot goes. Pose = (x inches, y inches, heading in radians)
    private final Pose startPose = new Pose(59, 8, Math.toRadians(180));
    private final Pose endPose   = new Pose(8,  8, Math.toRadians(180));

    // Around the hive (see the comment at the top for why these numbers)
    private final Pose hiveLeftLow   = new Pose(33, 59,  Math.toRadians(180));
    private final Pose hiveLeftHigh  = new Pose(33, 111, Math.toRadians(180));
    private final Pose hiveTopMiddle = new Pose(59, 111, Math.toRadians(180));
    private final Pose nearFlower    = new Pose(59, 133, Math.toRadians(180)); // adjust once the collector is built

    // Turn the collector on once the robot's X gets to this number or smaller.
    // (We're driving toward smaller X, so "<=" means "we've reached it or passed it".)
    private static final double COLLECTOR_START_X = 31;
    private static final double COLLECTOR_POWER = 0.5; // same as DriveTeleOp

    // How long to wait before starting. 0 = go right away. (Handy to raise in a
    // match if our alliance partner needs us to stay out of their way at first.)
    private static final double WAIT_SECONDS = 0;

    // Top speed while testing (1.0 = full speed). Slower = less overshoot and softer
    // hits if something goes wrong. Raise it once the robot stops where it should.
    private static final double MAX_POWER = 0.85;

    private Follower follower;    // Pedro Pathing: drives the robot along paths
    private DcMotorEx collector;
    private Timer pathTimer;      // measures time since the current step started
    private int pathState;        // which step we're on
    private boolean collectorOn = false;

    private Path startToEnd;
    private Path toHiveLeftLow, toHiveLeftHigh, toHiveTopMiddle, toNearFlower;

    private void buildPaths() {
        // Straight line left along the wall, keep facing 180 the whole way
        startToEnd = new Path(new BezierLine(startPose, endPose));
        startToEnd.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        // Around the hive. Constant heading = keep facing 180 the whole time.
        toHiveLeftLow = new Path(new BezierLine(endPose, hiveLeftLow));
        toHiveLeftLow.setConstantHeadingInterpolation(Math.toRadians(180));

        toHiveLeftHigh = new Path(new BezierLine(hiveLeftLow, hiveLeftHigh));
        toHiveLeftHigh.setConstantHeadingInterpolation(Math.toRadians(180));

        toHiveTopMiddle = new Path(new BezierLine(hiveLeftHigh, hiveTopMiddle));
        toHiveTopMiddle.setConstantHeadingInterpolation(Math.toRadians(180));

        toNearFlower = new Path(new BezierLine(hiveTopMiddle, nearFlower));
        toNearFlower.setConstantHeadingInterpolation(Math.toRadians(180));
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Sitting at the start. After the wait (if any), drive to the end.
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(startToEnd, true); // true = hold position at the end
                    setPathState(1);
                }
                break;
            case 1:
                // Driving. Turn the collector on once we pass x = 31.
                if (!collectorOn && follower.getPose().getX() <= COLLECTOR_START_X) {
                    collector.setPower(COLLECTOR_POWER);
                    collectorOn = true;
                }
                // isBusy() is true until the robot reaches the end of the path.
                if (!follower.isBusy()) {
                    collector.setPower(0);
                    collectorOn = false;
                    follower.followPath(toHiveLeftLow, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Each step below: wait until we arrive, then start the next move.
                if (!follower.isBusy()) {
                    follower.followPath(toHiveLeftHigh, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(toHiveTopMiddle, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toNearFlower, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    telemetry.speak("Done"); // spoken by the Driver Station
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

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.REVERSE); // same as RobotHardware

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // tell the robot where it is at the beginning
        follower.setMaxPower(MAX_POWER);
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
        telemetry.addData("Collector", collectorOn ? "ON" : "off");
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
