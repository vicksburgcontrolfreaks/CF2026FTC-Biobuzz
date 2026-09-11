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

import java.net.PasswordAuthentication;
// import com.bylazar.field.FieldManager; // Uncomment if available
// import com.bylazar.field.PanelsField; // Uncomment if available
// import com.bylazar.field.Style; // Uncomment if available
// import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Uncomment if you have this class

@Autonomous(name = "BlueShortShot", group = "Autonomous")
public class BlueShortShot extends OpMode {
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
    private Path blueSpike3Load;
    private Path blueShortScore1;
    private Path blueSpike2load;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(AutonConstants.blueShortStart, AutonConstants.blueSpike3));
        scorePreload.setLinearHeadingInterpolation(AutonConstants.blueShortStart.getHeading(), AutonConstants.blueSpike3.getHeading());

        blueSpike3Load = new Path(new BezierLine(AutonConstants.blueSpike3, AutonConstants.blueShortScore));
        blueSpike3Load.setLinearHeadingInterpolation(AutonConstants.blueSpike3.getHeading(), AutonConstants.blueShortScore.getHeading());

        blueShortScore1 = new Path(new BezierLine(AutonConstants.blueShortScore, AutonConstants.blueSpike2));
        blueShortScore1.setLinearHeadingInterpolation(AutonConstants.blueShortScore.getHeading(), AutonConstants.blueSpike2.getHeading());

        blueSpike2load = new Path(new BezierLine(AutonConstants.blueSpike2, AutonConstants.blueShortScore));
        blueSpike2load.setLinearHeadingInterpolation(AutonConstants.blueSpike2.getHeading(), AutonConstants.blueShortScore.getHeading());

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
                follower.followPath(preloadToLoad, true); // Hold endpoint
                setPathState(1);
                break;
            case 1:
                // Wait until at blueLongLoad
                if (!follower.isBusy()) {
                    follower.followPath(loadToScore, true); // Hold endpoint
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