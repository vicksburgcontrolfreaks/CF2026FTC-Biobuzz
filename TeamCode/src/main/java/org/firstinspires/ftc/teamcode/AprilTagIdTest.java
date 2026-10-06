package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;

import java.util.List;

@TeleOp(name = "AprilTag ID Test", group = "Test")
public class AprilTagIdTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Use the BIOBUZZ tags. The default library only knows past games' tags,
        // so it can't measure distance to ours.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FieldAprilTags.getLibrary())
                .setDrawTagOutline(true) // draw on the camera preview so you can see what it found
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData(">", "Touch START to start looking for tags");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Tags Detected", detections.size());
            for (AprilTagDetection detection : detections) {
                telemetry.addData("Tag ID", detection.id);
                if (detection.metadata != null) {
                    telemetry.addData("Which tag", detection.metadata.name);
                    telemetry.addData("Range (in)", "%.1f", detection.ftcPose.range);
                    telemetry.addData("Bearing (deg)", "%.1f", detection.ftcPose.bearing);
                    telemetry.addData("Elevation (deg)", "%.1f", detection.ftcPose.elevation);
                    telemetry.addData("X Y Z (in)", "%.1f, %.1f, %.1f",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                    telemetry.addData("Pitch Roll Yaw (deg)", "%.1f, %.1f, %.1f",
                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                } else {
                    telemetry.addData("Pose", "unavailable (not a BIOBUZZ tag)");
                }
            }
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }
}
