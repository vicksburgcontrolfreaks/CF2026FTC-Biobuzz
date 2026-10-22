package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * TurretAim: points the Turret at a hive cell using the AprilTags stuck to it.
 *
 * Feed it the detection list every loop and it will call turret.setTargetAngle()
 * when, and only when, that is the right thing to do. It never touches the servo
 * itself -- Turret.update() still owns that.
 *
 * WHY THIS IS NOT JUST setTargetAngle(bearing)
 *
 * The camera is bolted to the turret, so it moves with the thing it is aiming.
 * That makes bearing a RELATIVE measurement: "the tag is 12 degrees to my left
 * of wherever I happen to be pointing right now." Turn the turret and the number
 * changes. Chase it continuously and you have a slow, noisy camera loop running
 * inside the turret's own position loop -- two loops at different speeds, which
 * is exactly where oscillation comes from.
 *
 * So this class does NOT chase. It does SNAPSHOT -> COMMIT -> VERIFY:
 *
 *   1. Wait until the turret is sitting still (turret.isOnTarget()).
 *   2. Collect a few camera frames and average them into a smooth bearing.
 *   3. Turn that into ONE absolute command: setTargetAngle(getAngle() + bearing).
 *   4. Let the turret drive there on its ENCODER, blind. The camera is ignored
 *      the whole way, and the tag is allowed to slide out of frame mid-slew --
 *      the encoder knows where the turret is, so losing sight does not matter.
 *   5. On arrival, measure again. Still off? Take another step. Inside the
 *      deadband? Done.
 *
 * This works because the turret encoder is ABSOLUTE once homed. The camera only
 * has to be right at the start and the end of a move, never during it.
 *
 * WHY FRAMES TAKEN WHILE MOVING ARE THROWN AWAY
 *
 * getAngle() + bearing is only true if both halves were measured at the same
 * instant. A camera frame is already tens of milliseconds old by the time the
 * code sees it (watch getAgeMs()), and a turret covers real degrees in that
 * time. While the turret is slewing, the two halves disagree and the sum is
 * garbage. Standing still, a stale frame costs nothing -- nothing moved. Hence
 * step 1.
 *
 * THE DEADBAND IS THE MOST IMPORTANT NUMBER HERE
 *
 * AprilTag bearing readings jump a few degrees even with everything bolted down.
 * AprilTagFollow found that out on the robot and set BEARING_TOLERANCE = 5 deg
 * for the same reason. Turret.TOLERANCE_DEG is 2 deg, so the turret can settle
 * TIGHTER than the camera can see. Feed it raw bearing and it will chase camera
 * noise forever and look like a tuning problem.
 *
 *   DEADBAND_DEG must stay LARGER than Turret.TOLERANCE_DEG.
 *
 * If it is smaller, this class will command moves the turret considers "already
 * arrived", and the turret will sit there doing nothing while the status line
 * claims it is correcting.
 *
 * The bearing math is lifted from AprilTagFollow.whereIsTheTag() so both aim the
 * same way. It deliberately does not use ftcPose.bearing -- see bearingOf().
 */
@Configurable
public class TurretAim {

    // ---- Where the camera is ------------------------------------------------
    // How far the camera is tipped UP from level, in degrees. This matters: the
    // BIOBUZZ tags are on the BOTTOM of the hive cells facing DOWN, so the camera
    // has to look up at them. Measured by hand at about 30 deg on 2026-10-22 --
    // re-measure if the mount is touched.
    public static double CAMERA_PITCH_DEG = 30.0;

    // Dial in the difference between where the CAMERA looks and where the thing
    // you are actually aiming points. Leave at 0 while bringing this up; if the
    // turret reliably lands a few degrees to one side of the tag, put that here
    // rather than bending the bearing math.
    public static double AIM_BIAS_DEG = 0.0;

    // Set to -1 if the turret turns AWAY from the tag instead of toward it.
    // Same idea as Turret.ENCODER_DIRECTION: sign errors get ONE knob, and you
    // find out which way it goes in the first ten seconds on the robot. Do not
    // "fix" a sign by rearranging the atan2 in bearingOf().
    public static double BEARING_SIGN = 1.0;

    // ---- Noise handling -----------------------------------------------------
    // How much each NEW camera frame counts. 1.0 = no smoothing (jumpy),
    // 0.1 = very smooth but slow to notice the tag moved. 0.3 is what
    // AprilTagFollow settled on, on this robot, with this camera.
    public static double SMOOTHING = 0.3;

    // Close enough -- do not bother correcting. MUST be bigger than the camera's
    // noise AND bigger than Turret.TOLERANCE_DEG. See the class comment.
    public static double DEADBAND_DEG = 5.0;

    // How many NEW frames to average before acting on them. One frame is noise;
    // three is a measurement. Raise it if the first step of a correction tends
    // to go the wrong way.
    public static int FRAMES_BEFORE_COMMIT = 3;

    // Biggest single step this will command. A garbage frame cannot then throw
    // the turret across its whole travel. Corrections bigger than this just take
    // more than one step, which snapshot-and-commit already does anyway.
    public static double MAX_STEP_DEG = 60.0;

    // ---- Losing the tag -----------------------------------------------------
    // The turret HOLDS its last target when the tag disappears -- it does not
    // hunt. Searching is a separate feature; holding still is always safe.
    public static double LOST_HOLD_SEC = 0.5;

    /** Which tags count. Any of them seen at once get averaged together. */
    private int[] targetTagIds;

    // ---- State --------------------------------------------------------------
    private double smoothBearing = 0.0;
    private boolean smoothReady = false;
    private int framesAveraged = 0;
    private long lastFrameTime = 0;

    private double rawBearing = 0.0;
    private int tagsUsed = 0;
    private double ageMs = 0.0;
    private double lastCommittedTarget = 0.0;
    private boolean outOfReach = false;
    private String status = "idle";

    private final ElapsedTime sinceLastSeen = new ElapsedTime();

    public TurretAim(int[] targetTagIds) {
        this.targetTagIds = targetTagIds;
    }

    /** Switch which hive cell to aim at. See FieldAprilTags for the ID groups. */
    public void setTargetTagIds(int[] ids) {
        this.targetTagIds = ids;
        forgetMeasurement();
    }

    /**
     * Run every loop, BEFORE turret.update().
     *
     * @param detections what AprilTagProcessor.getDetections() just returned
     * @param turret     the turret to aim; only setTargetAngle() is ever called
     * @param enabled    false = look but never command. Telemetry still updates,
     *                   so you can watch what it WOULD do before letting it move.
     */
    public void update(List<AprilTagDetection> detections, Turret turret, boolean enabled) {
        // No angle frame means getAngle() + bearing is meaningless.
        if (!turret.isHomed()) {
            status = "turret not homed -- press X to home first";
            forgetMeasurement();
            return;
        }

        // --- 1. Average every cluster tag we can currently see ---------------
        double bearingSum = 0.0;
        int count = 0;
        long newestFrame = 0;

        for (AprilTagDetection tag : detections) {
            // metadata == null means the tag is not in FieldAprilTags, so the SDK
            // does not know its size and ftcPose is not filled in at all.
            if (tag.metadata == null || !isTargetTag(tag.id)) continue;
            bearingSum += bearingOf(tag);
            count++;
            if (tag.frameAcquisitionNanoTime > newestFrame) {
                newestFrame = tag.frameAcquisitionNanoTime;
            }
        }

        if (count == 0) {
            tagsUsed = 0;
            forgetMeasurement();
            status = sinceLastSeen.seconds() > LOST_HOLD_SEC
                    ? String.format("tag LOST %.1fs -- holding %.1f deg",
                            sinceLastSeen.seconds(), turret.getTargetAngle())
                    : "no tag this frame";
            return;
        }

        sinceLastSeen.reset();
        tagsUsed = count;
        // Plain average is fine: the four tags on one cell are a few degrees
        // apart at most, so there is no wrap-around to worry about. Averaging is
        // the cheapest noise reduction available -- four tags roughly halves it.
        rawBearing = bearingSum / count;
        ageMs = (System.nanoTime() - newestFrame) / 1.0e6;

        // --- 2. Ignore the camera entirely while the turret is moving --------
        if (!turret.isOnTarget()) {
            forgetMeasurement();
            status = String.format("slewing to %.1f -- camera ignored", turret.getTargetAngle());
            return;
        }

        // --- 3. Smooth, but only once per CAMERA frame -----------------------
        // This loop runs faster than the camera, so getDetections() hands back
        // the same frame several times. Smoothing it repeatedly would quietly
        // strengthen the filter -- applying 0.3 three times is really 0.66 --
        // and the amount would drift with loop speed. Count each frame once.
        boolean newFrame = newestFrame != lastFrameTime;
        lastFrameTime = newestFrame;

        if (!smoothReady) {
            smoothBearing = rawBearing;
            smoothReady = true;
            framesAveraged = 1;
        } else if (newFrame) {
            smoothBearing += SMOOTHING * (rawBearing - smoothBearing);
            framesAveraged++;
        }

        if (framesAveraged < FRAMES_BEFORE_COMMIT) {
            status = String.format("settling %d/%d frames", framesAveraged, FRAMES_BEFORE_COMMIT);
            return;
        }

        // --- 4. Already pointed at it? ---------------------------------------
        if (Math.abs(smoothBearing) <= DEADBAND_DEG) {
            outOfReach = false;
            status = String.format("ON TARGET (%.1f deg off, deadband %.1f)",
                    smoothBearing, DEADBAND_DEG);
            return;
        }

        // --- 5. Commit one absolute move -------------------------------------
        double step = Range.clip(smoothBearing * BEARING_SIGN, -MAX_STEP_DEG, MAX_STEP_DEG);
        double desired = turret.getAngle() + step + AIM_BIAS_DEG;

        // setTargetAngle() clips to the soft limits SILENTLY, which would leave
        // the turret parked at a limit, pointed at nothing, looking like it is
        // working. Say so instead.
        outOfReach = desired < Turret.MIN_ANGLE || desired > Turret.MAX_ANGLE;

        if (!enabled) {
            status = String.format("DISABLED -- would aim %.1f (off by %.1f)%s",
                    desired, smoothBearing, outOfReach ? "  [OUT OF REACH]" : "");
            return;
        }

        turret.setTargetAngle(desired);
        lastCommittedTarget = desired;
        // Force a fresh measurement after the move: the reading we just used was
        // taken at the OLD turret angle and is meaningless at the new one.
        forgetMeasurement();
        status = outOfReach
                ? String.format("OUT OF REACH -- wanted %.1f, limits are %.0f..%.0f",
                        desired, Turret.MIN_ANGLE, Turret.MAX_ANGLE)
                : String.format("committed %.1f deg (step %+.1f)", desired, step);
    }

    /**
     * Horizontal bearing to one tag, in degrees. Positive = the tag is to the
     * CAMERA's left.
     *
     * Same math as AprilTagFollow.whereIsTheTag(). ftcPose.bearing is not used
     * on purpose: it is measured in the camera's own tilted frame, and with the
     * camera pitched up 30 deg to see tags on the underside of a hive cell, that
     * is not the angle a vertical-axis turret needs. Rotating the camera's
     * forward axis back down to horizontal first is what makes the number mean
     * "how far do I have to spin".
     */
    private static double bearingOf(AprilTagDetection tag) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double forward = tag.ftcPose.y * Math.cos(pitch) - tag.ftcPose.z * Math.sin(pitch);
        double right = tag.ftcPose.x;
        return Math.toDegrees(Math.atan2(-right, forward));
    }

    private boolean isTargetTag(int id) {
        for (int target : targetTagIds) {
            if (target == id) return true;
        }
        return false;
    }

    /** Drop the averaged reading. The next commit waits for fresh frames. */
    private void forgetMeasurement() {
        smoothReady = false;
        framesAveraged = 0;
    }

    // ---- Telemetry ----------------------------------------------------------

    /** What this class is doing and why -- put it on the Driver Station. */
    public String getStatus() {
        return status;
    }

    /** Smoothed degrees off target. This is what a commit decision is made on. */
    public double getSmoothBearing() {
        return smoothBearing;
    }

    /** This frame's average, unsmoothed. Watch it jump with the turret still -- that is the noise. */
    public double getRawBearing() {
        return rawBearing;
    }

    /** How many of the cell's tags were visible. More = less noise. */
    public int getTagsUsed() {
        return tagsUsed;
    }

    /** Age of the newest frame, in ms -- how stale the bearing is. */
    public double getAgeMs() {
        return ageMs;
    }

    public double getLastCommittedTarget() {
        return lastCommittedTarget;
    }

    /** True when the tag is outside the turret's travel -- it physically cannot aim there. */
    public boolean isOutOfReach() {
        return outOfReach;
    }

    public boolean isOnTarget() {
        return smoothReady
                && framesAveraged >= FRAMES_BEFORE_COMMIT
                && Math.abs(smoothBearing) <= DEADBAND_DEG;
    }

    public double getSecondsSinceSeen() {
        return sinceLastSeen.seconds();
    }
}
