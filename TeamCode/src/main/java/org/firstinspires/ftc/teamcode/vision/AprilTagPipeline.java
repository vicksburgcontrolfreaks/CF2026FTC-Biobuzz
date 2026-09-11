// Updated AprilTagPipeline.java
package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import java.util.List;

public class AprilTagPipeline {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final double FX = 578.272;  // Calibrate these!
    private static final double FY = 578.272;
    private static final double CX = 402.145;
    private static final double CY = 221.506;

    // Known tag positions in Pedro coords (0-144 field, bottom-left origin)
    private static final Pose TAG_20 = new Pose(72, 0, Math.toRadians(90));    // Blue scoring
    private static final Pose TAG_24 = new Pose(72, 144, Math.toRadians(-90));  // Red scoring
    private static final Pose TAG_21 = new Pose(36, 72, 0);                    // Obelisk green/purple/purple
    private static final Pose TAG_22 = new Pose(36, 72, 0);                    // Obelisk pattern 2
    private static final Pose TAG_23 = new Pose(36, 72, 0);                    // Obelisk pattern 3

    public AprilTagPipeline(WebcamName webcam) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag.setDecimation(2);  // Adjust for speed/accuracy

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();
    }

    public void startStreaming() {
        if (!visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            visionPortal.resumeStreaming();
        }
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void stop() {
        visionPortal.close();
    }

    public void drawDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                AprilTagPoseFtc pose = detection.ftcPose;
                // Telemetry example: telemetry.addData("Tag ID", detection.id);
                // Add drawing logic if needed (requires OpenCV integration)
            }
        }
    }

    // Compute robot pose from tag detection
    public Pose getRobotPoseFromTag(AprilTagDetection detection) {
        if (detection.metadata == null || detection.ftcPose == null) return null;

        AprilTagPoseFtc tagPose = detection.ftcPose;
        Pose tagWorldPose = getTagWorldPose(detection.id); // Get known tag position
        if (tagWorldPose == null) return null;

        // Relative pose from camera to tag (x, y, z in camera frame)
        double relX = tagPose.x;
        double relY = tagPose.y;
        double relZ = tagPose.z;
        double relYaw = Math.toRadians(tagPose.yaw);

        // Transform to robot pose (simplified; use full pinhole camera model for accuracy)
        // Robot pose = tag pose - relative pose (adjust with camera extrinsics)
        double robotX = tagWorldPose.getX() - (relX * Math.cos(tagWorldPose.getHeading()) - relY * Math.sin(tagWorldPose.getHeading()));
        double robotY = tagWorldPose.getY() - (relX * Math.sin(tagWorldPose.getHeading()) + relY * Math.cos(tagWorldPose.getHeading()));
        double robotHeading = tagWorldPose.getHeading() - relYaw;

//         Normalize heading and convert to Pedro coords if needed (already in Pedro if tagWorldPose is)
//        robotHeading = MathFunctions.normalizeHeading(robotHeading);

        return new Pose(robotX, robotY, robotHeading);
    }

    // Get known tag position in Pedro coords
    private Pose getTagWorldPose(int tagId) {
        switch (tagId) {
            case 20: return TAG_20;
            case 24: return TAG_24;
            case 21: return TAG_21;
            case 22: return TAG_22;
            case 23: return TAG_23;
            default: return null;
        }
    }

    // Detect artifact pattern from obelisk tags
    public String getArtifactPattern() {
        List<AprilTagDetection> detections = getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 21) return "GREEN_PURPLE_PURPLE";
            if (detection.id == 22) return "PATTERN_2";
            if (detection.id == 23) return "PATTERN_3";
        }
        return "UNKNOWN";
    }
}