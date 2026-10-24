package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Shows everything the camera tells us about each AprilTag it sees, so we can
 * learn how far away and at what angles the BIOBUZZ tags are usable.
 *
 * Why a custom tag library: SDK 11.2.1's built-in library is last season's
 * (DECODE). The SDK can only compute ftcPose if it knows a tag's SIZE, and it
 * doesn't know the BIOBUZZ tags, so with the defaults most of them would show
 * up with no pose at all.
 *
 * BIOBUZZ tags are 3.25 in. (manual 9.9). We only know some of the IDs for
 * sure (38-41), so we register IDs 0-63 at that size. Only BIOBUZZ tags are
 * on the field, so the extra IDs do no harm. Once we read the real IDs off
 * Figure 9-17, trim this down.
 *
 * ftcPose is the tag's position RELATIVE TO THE CAMERA (not the field):
 *   x = right, y = forward (out of the lens), z = up   (inches)
 *   range = straight-line distance, bearing = left/right angle,
 *   elevation = up/down angle                          (inches / degrees)
 */
@TeleOp(name = "AprilTag ID Test", group = "Test")
public class AprilTagIdTest extends LinearOpMode {

    private static final double BIOBUZZ_TAG_SIZE_IN = 3.25;
    private static final int MAX_TAG_ID = 63;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id = 0; id <= MAX_TAG_ID; id++) {
            library.addTag(id, "BIOBUZZ " + id, BIOBUZZ_TAG_SIZE_IN, DistanceUnit.INCH);
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(library.build())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData(">", "Touch START to start looking for tags");
        telemetry.addData(">", "Camera stream: Driver Station menu > Camera Stream");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Tags Detected", detections.size());
            for (AprilTagDetection detection : detections) {
                telemetry.addLine();
                // Decision margin: how confident the detector is (higher = better).
                // Hamming: bits it had to correct (0 is best).
                telemetry.addLine(String.format("--- Tag %d  (margin %.0f, hamming %d)",
                        detection.id, detection.decisionMargin, detection.hamming));

                AprilTagPoseFtc pose = detection.ftcPose;
                if (detection.metadata == null || pose == null) {
                    // Not in our library, so the SDK doesn't know its size: no pose.
                    telemetry.addLine(String.format("  No pose (ID not in library). Pixel center %.0f, %.0f",
                            detection.center.x, detection.center.y));
                    continue;
                }
                telemetry.addLine(String.format("  Range %.1f in  Bearing %.1f°  Elev %.1f°",
                        pose.range, pose.bearing, pose.elevation));
                telemetry.addLine(String.format("  X %.1f  Y %.1f  Z %.1f in",
                        pose.x, pose.y, pose.z));
                telemetry.addLine(String.format("  Pitch %.1f  Roll %.1f  Yaw %.1f°",
                        pose.pitch, pose.roll, pose.yaw));
            }
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }
}
