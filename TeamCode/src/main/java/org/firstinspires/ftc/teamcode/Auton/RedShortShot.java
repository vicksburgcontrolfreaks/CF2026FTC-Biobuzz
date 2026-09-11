package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.Auton.AutonConstants;
// import com.bylazar.field.FieldManager; // Uncomment if available
// import com.bylazar.field.PanelsField; // Uncomment if available
// import com.bylazar.field.Style; // Uncomment if available
// import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Uncomment if you have this class

@Autonomous(name = "RedShortShot", group = "Autonomous")
public class RedShortShot extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private PathChain preloadToLoad, loadToScore;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Build paths
        buildPaths();

        // Initialize dashboard (comment out if FieldManager/PanelsField unavailable)
        // Drawing.init();

        telemetry.update();
    }
    private Path scorePreload;
    private Path redSpike3Load;
    private Path redShortScore1;
    private Path redSpike2Load;
    private void buildPaths() {

        scorePreload = new Path(new BezierLine(AutonConstants.redShortStart, AutonConstants.redSpike3));
        scorePreload.setLinearHeadingInterpolation(AutonConstants.redShortStart.getHeading(), AutonConstants.redSpike3.getHeading());

        redSpike3Load = new Path(new BezierLine(AutonConstants.redSpike3, AutonConstants.redShortScore));
        redSpike3Load.setLinearHeadingInterpolation(AutonConstants.redSpike3.getHeading(), AutonConstants.redShortScore.getHeading());

        redShortScore1 = new Path(new BezierLine(AutonConstants.redShortScore, AutonConstants.redSpike2));
        redShortScore1.setLinearHeadingInterpolation(AutonConstants.redShortScore.getHeading(), AutonConstants.redSpike2.getHeading());

        redSpike2Load = new Path(new BezierLine(AutonConstants.redSpike2, AutonConstants.redShortScore));
        redSpike2Load.setLinearHeadingInterpolation(AutonConstants.redSpike2.getHeading(), AutonConstants.redShortScore.getHeading());

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
            // Drawing.drawDebug(follower); // Uncomment if dashboard available
            autonomousPathUpdate();

            // Debug telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Shoot from blueLongStart
                shoot();
                follower.followPath(scorePreload, true); // Hold endpoint
                setPathState(1);
                break;
            case 1:
                // Wait until at blueLongLoad
                if (!follower.isBusy()) {
                    follower.followPath(redShortScore1, true); // Hold endpoint
                    setPathState(2);
                }
                break;
            case 2:
                // Shoot from blueLongScore
                if (!follower.isBusy()) {
                    shoot();
                    setPathState(-1); // Stop
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void shoot() {
        // Replace with your motor/servo code, e.g.:
        // DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        // shooter.setPower(0.8);
        // sleep(1000); // Run for 1 sec
        // shooter.setPower(0);
        try {
            Thread.sleep(1000); // Placeholder delay
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Shoot interrupted: " + e.getMessage());
        }
        telemetry.addData("Action", "Shot");
    }

    @Override
    public void stop() {
    }

    // Visualization (uncomment if FieldManager/PanelsField available)
    /*
    static class Drawing {
        public static final double ROBOT_RADIUS = 9;
        private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
        private static final Style robotLook = new Style("", "#3F51B5", 0.0);
        private static final Style historyLook = new Style("", "#4CAF50", 0.0);

        public static void init() {
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }

        public static void drawDebug(Follower follower) {
            if (follower.getCurrentPath() != null) {
                drawPath(follower.getCurrentPath(), robotLook);
                Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
                drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
            }
            drawRobot(follower.getPose(), historyLook);
            sendPacket();
        }

        public static void drawRobot(Pose pose, Style style) {
            if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
                return;
            }
            panelsField.setStyle(style);
            panelsField.moveCursor(pose.getX(), pose.getY());
            panelsField.circle(ROBOT_RADIUS);
            Vector v = pose.getHeadingAsUnitVector();
            v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
            double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
            double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
            panelsField.setStyle(style);
            panelsField.moveCursor(x1, y1);
            panelsField.line(x2, y2);
        }

        public static void drawPath(Path path, Style style) {
            double[][] points = path.getPanelsDrawingPoints();
            for (int i = 0; i < points[0].length; i++) {
                for (int j = 0; j < points.length; j++) {
                    if (Double.isNaN(points[j][i])) {
                        points[j][i] = 0;
                    }
                }
            }
            panelsField.setStyle(style);
            panelsField.moveCursor(points[0][0], points[0][1]);
            panelsField.line(points[1][0], points[1][1]);
        }

        public static void sendPacket() {
            panelsField.update();
        }
    }
    */
}