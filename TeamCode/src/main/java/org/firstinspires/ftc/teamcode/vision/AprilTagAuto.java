// Updated AprilTagAuto.java
package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auton.AutonConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing; // Import existing Drawing class
import java.util.List;

@Autonomous(name = "AprilTag Auto")
public class AprilTagAuto extends OpMode {
    private AprilTagPipeline pipeline;
    private WebcamName webcam;
    private Follower follower;
    private String artifactPattern = "UNKNOWN";

    @Override
    public void init() {
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        pipeline = new AprilTagPipeline(webcam);
        pipeline.startStreaming();

        // Initialize Pedro Follower with Pinpoint
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(AutonConstants.blueLongStart); // Example start pose

        // Initialize Panels Drawing
//        Drawing.init();
    }

    @Override
    public void loop() {
        follower.update();

        List<AprilTagDetection> detections = pipeline.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id > 0) {
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Tag Pose X", detection.ftcPose.x);
                telemetry.addData("Tag Pose Y", detection.ftcPose.y);
                telemetry.addData("Tag Yaw", detection.ftcPose.yaw);

                // Compute robot pose from tag
                Pose robotPose = pipeline.getRobotPoseFromTag(detection);
                if (robotPose != null) {
                    // Relocalize with scoring tags (20 or 24) if close
                    if ((detection.id == 20 || detection.id == 24) && detection.ftcPose.range < 36) {
                        follower.setPose(robotPose);
                        telemetry.addData("Relocalized Pose", robotPose.toString());
                    }

                    // Detect obelisk pattern
                    if (artifactPattern.equals("UNKNOWN")) {
                        artifactPattern = pipeline.getArtifactPattern();
                        telemetry.addData("Artifact Pattern", artifactPattern);
                    }

                    // Align for shooting if near scoring tag
                    if ((detection.id == 20 || detection.id == 24) && detection.ftcPose.range < 24) {
                        AprilTagPoseFtc tagPose = detection.ftcPose;
                        double targetHeading = (detection.id == 20) ? Math.toRadians(135) : Math.toRadians(-135);
                        Pose alignPose = new Pose(robotPose.getX(), robotPose.getY(), targetHeading);
                        follower.followPath(follower.pathBuilder()
                                .addPath(new com.pedropathing.geometry.BezierLine(robotPose, alignPose))
                                .setLinearHeadingInterpolation(robotPose.getHeading(), targetHeading)
                                .build(), true);
                        telemetry.addData("Aligning", "To Goal");
                    }
                }
            }
        }

        // Draw robot and current path on Panels field widget
//        Drawing.drawRobot(follower.getPose()); // Circle with bisecting line for direction
//        if (follower.getCurrentPath() != null) {
//            Drawing.drawPath(follower.getCurrentPath(), Drawing.robotLook);
//        }
//        Drawing.sendPacket(); // Update the field widget
//
//        telemetry.update();
    }

    @Override
    public void stop() {
        pipeline.stop();
    }
}