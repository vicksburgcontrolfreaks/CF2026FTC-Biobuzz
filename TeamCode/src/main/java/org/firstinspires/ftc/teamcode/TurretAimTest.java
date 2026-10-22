package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Bring-up op mode for turret AprilTag aiming.
 *
 * The camera ("Webcam 1") is mounted ON the turret, tipped up about 30 deg so it
 * can see the tags on the underside of a hive cell.
 *
 * AIMING STARTS DISABLED ON PURPOSE. Press START and the op mode watches the tag
 * and reports what it WOULD do -- "would aim -37.2 (off by -12.4)" -- without
 * commanding anything. That is how you check the sign before the turret is
 * allowed to move. Only press A once that line looks right.
 *
 * WHICH HIVE IT AIMS AT IS WORKED OUT FOR YOU. Point the camera at a hive and
 * the "Aiming at" line names it -- "Blue audience [38, 39, 40, 41] (AUTO)". No
 * need to remember which IDs belong to which cell, and no D-pad hunting. The
 * D-pad is still there to force a specific cell, and using it switches to manual
 * so auto-detect cannot pull you back; set AUTO_CELL in Panels to hand control
 * back to the camera.
 *
 * Controls (gamepad1):
 *   X             - start homing. Nothing else works until this finishes.
 *   A             - toggle aiming ON / OFF (starts OFF)
 *   B             - aiming OFF and send the turret back to 0 (straight ahead)
 *   D-pad L/R     - force a specific hive cell (turns AUTO_CELL off)
 *
 * BRING-UP ORDER
 *
 * 1. Robot on blocks, turret free, finger on stop.
 *
 * 2. Press X and let it home. The turret parks at 0. If homing fails, read the
 *    Note line and go back to docs/turret-bringup.md Step 6 -- aiming cannot work
 *    before this does, because "getAngle() + bearing" needs a real angle.
 *
 * 3. Point the camera somewhere it can see a cell. "Aiming at" should name the
 *    hive you are looking at, "Camera sees" should agree with it, and "Tags used"
 *    should go above 0. Check "Frame age" is a sane number of milliseconds.
 *
 *    If "Aiming at" says (manual) and disagrees with "Camera sees", someone has
 *    pressed the D-pad -- turn AUTO_CELL back on in Panels.
 *
 * 4. WITH AIMING STILL OFF, hold the turret so the tag is off to one side and
 *    read the "would aim" line.
 *      - "would aim" moves TOWARD the tag  -> BEARING_SIGN is right.
 *      - "would aim" moves AWAY from it    -> set BEARING_SIGN = -1 in Panels.
 *    This is the whole reason aiming starts disabled. Do not skip it: a wrong
 *    sign drives the turret into a hard stop at full closed-loop power.
 *
 * 5. Watch "Bearing raw" with everything held still. It will jump around by a
 *    few degrees -- that is the camera noise DEADBAND_DEG has to exceed. If raw
 *    swings more than 5 deg, raise DEADBAND_DEG to match before enabling.
 *
 * 6. Press A. The turret should take one step, settle, and report ON TARGET.
 *    Expect discrete steps, not a smooth glide -- that is snapshot-and-commit
 *    working as designed, not a bug.
 *
 * WHAT GOOD LOOKS LIKE
 *
 *   - Converges in one or two steps, then sits still and says ON TARGET.
 *   - Moving the robot makes it re-aim in steps and settle again.
 *   - Briefly blocking the camera makes it HOLD its last angle, not wander.
 *   - Blocking it for longer puts Phase into SEARCHING: the turret steps across
 *     its travel pausing to look, stops the moment it sees any hive, and hands
 *     back to TRACKING. Cover the camera completely and it finishes the sweep,
 *     goes PARKED at 0 deg, and waits there.
 *
 * TESTING THE SEARCH
 *
 * Put a hand over the lens for two seconds. Expect: hold, then stepping. Uncover
 * it pointed at a different hive and -- with AUTO_CELL on -- it should stop,
 * rename the cell and aim at the new one.
 *
 * The sweep is deliberately steppy. A smooth slew blurs tags badly enough that
 * the processor stops finding them, so stopping to look actually finds more.
 *
 * WHAT TO DO IF IT MISBEHAVES
 *
 *   - Hunts back and forth forever, never says ON TARGET
 *       -> DEADBAND_DEG is below the camera noise. Raise it. It MUST also stay
 *          above Turret.TOLERANCE_DEG (2.0) or commits get swallowed.
 *   - Overshoots and comes back
 *       -> Turret.kP is high for this, or MAX_STEP_DEG is letting one bad frame
 *          through. Lower MAX_STEP_DEG first, it is the safer knob.
 *   - First step goes the wrong way, then it recovers
 *       -> FRAMES_BEFORE_COMMIT is too low. Raise it to 5.
 *   - Says OUT OF REACH
 *       -> the tag is genuinely outside +/-140 deg of turret travel. Not a bug.
 *          The robot has to turn; the turret cannot get there.
 *
 * Every constant on Turret and TurretAim is live-tunable at panels.bylazar.com
 * while this runs. Panels edits are NOT saved -- write down what works.
 */
@Configurable
@TeleOp(name = "Turret Aim Test", group = "Test")
public class TurretAimTest extends LinearOpMode {

    // Short exposure = less motion blur. This matters more than it does for a
    // chassis camera: a turret whips the camera through degrees fast, and a
    // blurred tag is a tag the processor does not find at all. Raise toward
    // 10-15 only if the stream is too dark to see the tag while standing still.
    public static int EXPOSURE_MS = 6;
    public static int CAMERA_GAIN = 250;

    // Look at every Nth pixel. 2 is faster but sees less far. The hive tags are
    // small (3.25 in.), so drop to 1 if it cannot find them from across the tile.
    public static int DECIMATION = 2;

    // Work out which hive cell we are looking at from the tags in frame, instead
    // of making someone remember that 38-41 is the Blue audience hive. Point the
    // camera at a hive and it names it and aims at it.
    //
    // Touching the D-pad turns this OFF, because an explicit request for a cell
    // should not get silently overridden a moment later.
    //
    // MATCH CODE SHOULD NOT DO THIS. There, set the cell from the alliance you
    // are actually on, before the match starts. Retargeting to whichever hive
    // wanders into frame is a great way to score for the other alliance.
    public static boolean AUTO_CELL = true;

    private final Turret turret = new Turret();
    private TurretAim aim;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TelemetryManager telemetryM;

    // Which cell to aim at. Hive tags come in clusters of four; TurretAim
    // averages whichever of them it can see, which roughly halves the noise.
    private FieldAprilTags.Cell cell = FieldAprilTags.Cell.RED_FAR;
    /** The cell the camera can actually see right now, or null if it sees none. */
    private FieldAprilTags.Cell seenCell = null;

    private boolean aiming = false;
    private boolean aPrev, bPrev, xPrev, dpadLeftPrev, dpadRightPrev;

    @Override
    public void runOpMode() {
        turret.init(hardwareMap);
        aim = new TurretAim(cell.tagIds);

        initCamera();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(turret);
        PanelsConfigurables.INSTANCE.refreshClass(aim);

        telemetry.addLine("TURRET AIM TEST");
        telemetry.addLine("X = home  |  A = aiming on/off  |  B = park at 0");
        telemetry.addLine("D-pad L/R = force a hive cell (otherwise automatic)");
        telemetry.addLine("");
        telemetry.addLine("The hive is identified from the tags in frame --");
        telemetry.addLine("just point the camera at it and read 'Aiming at'.");
        telemetry.addLine("Aiming starts OFF. Home first (X), then watch the");
        telemetry.addLine("'would aim' line to check BEARING_SIGN before pressing A.");
        telemetry.addLine("Camera view: 3 dots menu > Camera Stream");
        telemetry.update();

        waitForStart();
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);

        while (opModeIsActive()) {
            handleButtons();

            List<AprilTagDetection> detections = aprilTag.getDetections();

            // Name the hive in front of us, and adopt it unless someone has made
            // an explicit choice with the D-pad.
            seenCell = dominantCell(detections);
            if (AUTO_CELL && seenCell != null && seenCell != cell) {
                cell = seenCell;
                aim.setTargetTagIds(cell.tagIds);
            }

            // Order matters: decide the target first, then let the turret act on
            // it in the same loop. TurretAim only ever calls setTargetAngle();
            // turret.update() is still the only thing that writes to the servo.
            aim.update(detections, turret, aiming);
            turret.update();

            showTelemetry(detections.size());
        }

        turret.stop();
        visionPortal.close();
    }

    private void handleButtons() {
        if (gamepad1.x && !xPrev) {
            turret.startHoming();
            aiming = false;   // never aim through a homing run
        }
        xPrev = gamepad1.x;

        if (gamepad1.a && !aPrev) {
            aiming = !aiming;
            // Do not resume a sweep that was running when aiming was switched
            // off -- the driver gets a turret that holds still until it has
            // actually lost the tag again.
            aim.resetPhase();
        }
        aPrev = gamepad1.a;

        if (gamepad1.b && !bPrev) {
            aiming = false;
            aim.resetPhase();
            if (turret.isHomed()) turret.setTargetAngle(0.0);
        }
        bPrev = gamepad1.b;

        if (gamepad1.dpad_right && !dpadRightPrev) selectCell(+1);
        dpadRightPrev = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !dpadLeftPrev) selectCell(-1);
        dpadLeftPrev = gamepad1.dpad_left;
    }

    /**
     * Manual cell choice. Turns AUTO_CELL off: if you have asked for a specific
     * hive, auto-detect must not quietly pull you back to whatever is in frame.
     * Flip AUTO_CELL back on in Panels to hand control back to the camera.
     */
    private void selectCell(int step) {
        FieldAprilTags.Cell[] all = FieldAprilTags.Cell.values();
        cell = all[(cell.ordinal() + step + all.length) % all.length];
        aim.setTargetTagIds(cell.tagIds);
        AUTO_CELL = false;
    }

    /**
     * Which hive cell the camera is mostly looking at, or null if it sees none.
     *
     * Counts tags per cell and takes the winner rather than trusting the first
     * detection in the list, so one stray tag from the next hive over at the
     * edge of frame cannot steal the aim.
     */
    private FieldAprilTags.Cell dominantCell(List<AprilTagDetection> detections) {
        FieldAprilTags.Cell[] all = FieldAprilTags.Cell.values();
        int[] counts = new int[all.length];

        for (AprilTagDetection tag : detections) {
            if (tag.metadata == null) continue;   // not a BIOBUZZ tag, no pose
            FieldAprilTags.Cell found = FieldAprilTags.Cell.forTag(tag.id);
            if (found != null) counts[found.ordinal()]++;
        }

        FieldAprilTags.Cell best = null;
        int bestCount = 0;
        for (int i = 0; i < all.length; i++) {
            if (counts[i] > bestCount) {
                bestCount = counts[i];
                best = all[i];
            }
        }
        return best;
    }

    private void showTelemetry(int totalDetections) {
        telemetry.addData("Turret", "%s / %s", turret.getState(), turret.getControlMode());
        telemetry.addData("Note", turret.getStatusNote());
        telemetry.addData("Angle (deg)", "%+7.2f   target %+7.2f   onTarget %s",
                turret.getAngle(), turret.getTargetAngle(), turret.isOnTarget());
        telemetry.addLine("");

        telemetry.addData("AIMING", aiming ? ">>> ON <<<" : "off (press A)");
        telemetry.addData("Aiming at", "%s %s  %s", cell.displayName,
                java.util.Arrays.toString(cell.tagIds),
                AUTO_CELL ? "(AUTO)" : "(manual - D-pad picked it)");
        telemetry.addData("Camera sees", seenCell == null
                ? "no hive tags" : seenCell.displayName);
        telemetry.addData("Phase", "%s%s", aim.getPhase(),
                TurretAim.SEARCH_ENABLED ? "" : "  (SEARCH_ENABLED off)");
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addLine("");

        // "raw" is this frame; "smooth" is what a commit is decided on. Watch raw
        // jump with everything still -- that gap is why DEADBAND_DEG exists.
        telemetry.addData("Bearing smooth", "%+6.2f deg  (deadband %.1f)",
                aim.getSmoothBearing(), TurretAim.DEADBAND_DEG);
        telemetry.addData("Bearing raw", "%+6.2f deg", aim.getRawBearing());
        telemetry.addData("Tags used", "%d of %d visible", aim.getTagsUsed(), totalDetections);
        telemetry.addData("Frame age", "%.0f ms  (how stale the bearing is)", aim.getAgeMs());
        telemetry.addData("Last seen", "%.1f s ago", aim.getSecondsSinceSeen());
        if (aim.isOutOfReach()) {
            telemetry.addLine("");
            telemetry.addLine(">>> OUT OF REACH: tag is outside the turret's travel.");
            telemetry.addLine(">>> Turn the robot -- the turret cannot get there.");
        }

        telemetryM.addData("State", turret.getState());
        telemetryM.addData("Angle (deg)", turret.getAngle());
        telemetryM.addData("Target (deg)", turret.getTargetAngle());
        telemetryM.addData("Bearing smooth", aim.getSmoothBearing());
        telemetryM.addData("Bearing raw", aim.getRawBearing());
        telemetryM.addData("Tags used", aim.getTagsUsed());
        telemetryM.addData("Frame age (ms)", aim.getAgeMs());
        telemetryM.addData("Aim status", aim.getStatus());
        telemetryM.update(telemetry);

        telemetry.update();
    }

    private void initCamera() {
        // The BIOBUZZ tags are not in the SDK's built-in library, so without this
        // the processor reports IDs but no pose at all -- and no pose means no
        // bearing, which means no aiming.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FieldAprilTags.getLibrary())
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();
        // Cast matters: setDecimation(int) is deprecated, setDecimation(float) is not.
        aprilTag.setDecimation((float) DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /** Same approach as AprilTagFollow: manual short exposure to cut motion blur. */
    private void setManualExposure(int exposureMS, int gain) {
        telemetry.addLine("Camera starting...");
        telemetry.update();
        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (isStopRequested()) return;

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        visionPortal.getCameraControl(GainControl.class).setGain(gain);
        sleep(20);
    }
}
