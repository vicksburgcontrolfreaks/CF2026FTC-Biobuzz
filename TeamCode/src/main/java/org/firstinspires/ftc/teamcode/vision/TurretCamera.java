package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * The turret camera ("Webcam 1"), set up for BIOBUZZ hive tags, for op modes
 * that can't stop and wait (DriveTeleOp). Same settings as Turret Aim Test,
 * which is where they were tested on 6494 (2026-11-06):
 *
 *   - FieldAprilTags library. Without it the SDK knows the IDs but not the tag
 *     size, so there's no pose and no bearing -- no aiming.
 *   - Short manual exposure + high gain, to cut motion blur on a moving turret.
 *
 * Turret Aim Test sets exposure with sleep()s. This does the same steps spread
 * over update() calls instead, so the loop never stalls. Call update() every
 * loop (INIT too -- the camera usually finishes starting during INIT).
 *
 * If you change a number here, change it in Turret Aim Test too (or better,
 * point Turret Aim Test at these).
 */
public class TurretCamera {

    public static int EXPOSURE_MS = 6;
    public static int CAMERA_GAIN = 250;
    public static int DECIMATION = 2;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;

    private enum Setup { WAIT_STREAMING, MODE_SET, DONE, FAILED }
    private Setup setup = Setup.WAIT_STREAMING;
    private final ElapsedTime setupTimer = new ElapsedTime();
    private String setupNote = "camera starting";

    public TurretCamera(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FieldAprilTags.getLibrary())
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();
        aprilTag.setDecimation((float) DECIMATION);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /** Every loop. Finishes the exposure setup once the camera is streaming. */
    public void update() {
        try {
            switch (setup) {
                case WAIT_STREAMING:
                    if (portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                        if (exposure.getMode() != ExposureControl.Mode.Manual) {
                            exposure.setMode(ExposureControl.Mode.Manual);
                        }
                        setupTimer.reset();
                        setup = Setup.MODE_SET;
                    }
                    break;
                case MODE_SET:
                    if (setupTimer.milliseconds() >= 50) {   // same settle as Turret Aim Test
                        portal.getCameraControl(ExposureControl.class)
                                .setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                        portal.getCameraControl(GainControl.class).setGain(CAMERA_GAIN);
                        setup = Setup.DONE;
                        setupNote = "ready";
                    }
                    break;
                default:
                    break;
            }
        } catch (RuntimeException e) {
            // A camera control failing must not kill the match op mode. Aiming
            // still works on auto exposure, just with more blur.
            setup = Setup.FAILED;
            setupNote = "exposure setup failed (" + e.getMessage() + ") - auto exposure";
        }
    }

    /** This loop's hive tag detections (empty until streaming). */
    public List<AprilTagDetection> getDetections() {
        List<AprilTagDetection> d = aprilTag.getDetections();
        return d == null ? Collections.<AprilTagDetection>emptyList() : d;
    }

    public boolean isReady() { return setup == Setup.DONE; }
    public String getNote() { return setupNote; }

    public void close() {
        portal.close();
    }
}
