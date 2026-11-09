package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

// Hidden from the Driver Station 2026-11-05 (op mode cleanup): AprilTag ID Test covers the camera check.
// The code stays; delete this @Disabled line to bring it back.
@TeleOp(name = "Camera Test", group = "Test")
public class CameraTest extends LinearOpMode {
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080); // Match your XML resolution
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Camera Status", "Streaming");
            telemetry.update();
        }
        camera.stopStreaming();
    }
}