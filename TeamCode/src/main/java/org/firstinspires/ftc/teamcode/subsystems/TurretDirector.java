package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.geometry.AimLead;
import org.firstinspires.ftc.teamcode.geometry.OdometryAim;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/*
 * TurretDirector: the ONE place that decides how the turret is aimed.
 *
 * Before this, every op mode talked to TurretAim directly, so every op mode
 * aimed camera-only. This owns the choice instead, so DriveTeleOp, RouteAuto and
 * PollenShotTune all aim the same way and there is one thing to change.
 *
 * THREE BEHAVIOURS, FROM TWO FLAGS:
 *
 *   USE_ODOMETRY_AIM = false  CAMERA ONLY. What TurretAim did on its own,
 *                             unchanged. THE DEFAULT, because it is what has
 *                             actually been driven in matches.
 *   true, trim off            ODOMETRY ONLY. Aim from the pose, continuously,
 *                             with the turret lead. Ignores the camera.
 *   true, trim on             ODOMETRY, CAMERA-CORRECTED. The target design.
 *
 * WHY ODOMETRY DRIVES AND THE CAMERA CORRECTS
 *
 * They fail in opposite ways, which is the whole reason to combine them.
 *
 * Odometry is smooth, instant, and never loses sight of anything -- it can aim
 * at a hive cell that is behind a wall. But it drifts, and a bump knocks it off.
 *
 * The camera cannot drift, because a tag is where a tag is. But it is slow,
 * noisy, only works pointed at a visible cell, and TurretAim deliberately
 * ignores it while the turret is moving, so it cannot track continuously.
 *
 * So odometry runs the turret every loop, and whenever the camera DOES have a
 * settled bearing, the difference between what the camera says and what the pose
 * says is the pose's angular error. Hold that as a slowly filtered offset -- the
 * TRIM -- and add it to the odometry aim. The trim keeps working after the tag
 * leaves view, which is most of its value: the correction outlives the sighting.
 *
 * WHAT THE TRIM IS AND IS NOT
 *
 * It corrects the ANGLE, not the position. A heading error in the pose is a
 * constant angular error, so the trim cancels it exactly. A POSITION error shows
 * up as an angular error too, but one that changes as the robot drives, so the
 * trim only approximates it -- which is why it is re-estimated continuously
 * rather than latched. Fixing position properly needs TagLocalizer feeding
 * VisionPoseFilter into the pose itself. That is a bigger job and this does not
 * attempt it.
 *
 * It also does NOT fix range. Odometry supplies the distance, and a trimmed
 * angle does not make that distance righter.
 *
 * WHAT ODOMETRY AIMING BUYS THAT CAMERA AIMING NEVER DID: a DISTANCE to the
 * target, every loop, whether or not a tag is in view. ShotTable needs exactly
 * that, so this is also the thing that unblocks shooter tuning.
 *
 * NOTHING HERE CHANGES BEHAVIOUR UNTIL USE_ODOMETRY_AIM IS TURNED ON.
 */
@Configurable
public class TurretDirector {

    /**
     * false = camera-only aiming, what TurretAim did alone.
     *
     * DEFAULT FALSE on purpose: odometry aiming is only as good as the pose, and
     * the pose wants retuning after the current mechanical changes.
     *
     * WHO SETS THIS. RobotProfile sets it per robot at op-mode init -- but that is
     * only the DEFAULT for the "Aim by" row in the INIT menu, which writes it
     * again at Start from whatever the drive team picked. See AimSource. Changing
     * which way a robot aims should not need a redeploy while the pose is still
     * being chased.
     */
    public static boolean USE_ODOMETRY_AIM = false;

    /**
     * Where the turret's aim comes from, as the INIT menu offers it.
     *
     * CAMERA is the AprilTags: the turret only knows where the hive is when a tag
     * is in frame. ODOMETRY is the pose: it tracks continuously whether or not a
     * tag is visible, and is therefore only as good as the pose underneath it.
     *
     * This is a menu row and not just the boolean so the Driver Station reads
     * "camera (tags)" instead of "false", which nobody should have to translate
     * in the pits.
     */
    public enum AimSource {
        CAMERA("camera (tags)"),
        ODOMETRY("odometry (pose)");

        private final String label;

        AimSource(String label) {
            this.label = label;
        }

        /** What RobotProfile's default works out to. Read AFTER RobotProfile.install(). */
        public static AimSource fromDefault() {
            return USE_ODOMETRY_AIM ? ODOMETRY : CAMERA;
        }

        /**
         * Can this robot actually do it? ODOMETRY needs TURRET_OFFSET_X/Y measured
         * -- without them TurretDirector falls straight back to the camera, so
         * offering the option would be a lie. Feed this to the menu's allowWhen().
         */
        public boolean isAvailable() {
            return this != ODOMETRY || TurretGeometry.forRobot().isConfigured();
        }

        /** Make this the live choice. Call once at Start, not every loop. */
        public void apply() {
            USE_ODOMETRY_AIM = this == ODOMETRY;
        }

        @Override
        public String toString() {
            return label;
        }
    }

    /** Let the camera correct the odometry aim angle. Only read when aiming from odometry. */
    public static boolean USE_CAMERA_TRIM = true;

    /**
     * One INIT-menu row for the aim diagnostics that otherwise live only in
     * Panels, so they can be switched from the Driver Station D-pad instead.
     *
     * WHY A PRESET AND NOT TWO BOOLEAN ROWS. These are only ever flipped in a
     * known combination while chasing a bias, and two extra rows in the match
     * menu is two more things to scroll past before every match. One row, and
     * NORMAL is the default.
     *
     * NORMAL actively writes both flags back to their shipped defaults, so a run
     * can't inherit a leftover setting from an earlier run in the same power-up.
     *
     * Added 2026-11-12 to test the left-bias suspect: TurretAim.bearingOf()
     * subtracts HiveTags.rowOffsetOf(), a hive-frame offset, straight from a
     * camera-frame x with no account of which side the row is viewed from.
     */
    public enum AimTest {
        NORMAL("normal", true, true),
        NO_CELL_CENTRING("no cell centring", false, true),
        NO_CAMERA_TRIM("no camera trim", true, false),
        NEITHER("no centring, no trim", false, false);

        private final String label;
        private final boolean centreOnCell;
        private final boolean cameraTrim;

        AimTest(String label, boolean centreOnCell, boolean cameraTrim) {
            this.label = label;
            this.centreOnCell = centreOnCell;
            this.cameraTrim = cameraTrim;
        }

        /** Make this the live setting. Call once at Start, next to AimSource.apply(). */
        public void apply() {
            TurretAim.CENTRE_ON_CELL = centreOnCell;
            USE_CAMERA_TRIM = cameraTrim;
        }

        @Override
        public String toString() {
            return label;
        }
    }

    /** How fast the trim follows the camera. 0.15 = 15% of the disagreement per measurement. */
    public static double TRIM_SMOOTHING = 0.15;

    /**
     * Disagreements bigger than this are not believed. A pose really this far out
     * at shooting range is a failure for pose correction to fix rather than
     * something to quietly paper over, and the likelier explanation is a bad
     * frame or the wrong cell in view.
     */
    public static double MAX_TRIM_DEG = 15.0;

    /** A bearing from a frame older than this describes where we WERE. */
    public static double TRIM_MAX_AGE_MS = 150;

    /**
     * How long the camera and the pose have to disagree by more than
     * MAX_TRIM_DEG before odometry aiming is dropped for camera aiming.
     *
     * Long enough that one wild detection can't throw away a good pose, short
     * enough that a drifted pose doesn't get to aim a whole volley. At 0.75 s a
     * driver feels the turret hand over rather than watching it sit wrong.
     */
    public static double TRIM_DISTRUST_SEC = 0.75;

    /** Stop trusting the bearing this long after a tag was last seen. */
    public static double TRIM_FRESH_SEC = 0.5;

    /** Passed to OdometryAim: how far past the dead zone before switching limits. */
    public static double HYSTERESIS_DEG = 15.0;

    private final TurretAim camera;
    private final AimLead lead = new AimLead();

    private OdometryAim.Solution solution;
    private double trimDeg = 0;
    private boolean trimEverSet = false;
    private boolean onTarget = false;
    private String status = "not updated yet";
    private String source = "-";

    // ---- Is the pose worth aiming from? (2026-11-12) ------------------------
    /**
     * False once the camera and the pose have disagreed past MAX_TRIM_DEG for
     * TRIM_DISTRUST_SEC. While false the CAMERA aims and odometry is only
     * monitored, so a pose that comes good again can take the turret back.
     *
     * Starts true: with no measurement yet there is nothing against the pose.
     */
    private boolean poseTrusted = true;
    /** True while this frame's disagreement is over the cap. For telemetry. */
    private boolean rejecting = false;
    /** When the current run of rejections began, or NaN if we are not rejecting. */
    private double rejectingSinceSec = Double.NaN;
    /** The last over-cap disagreement, degrees. What the Driver Station reports. */
    private double lastRejectedDeg = 0;

    public TurretDirector(int[] targetTagIds) {
        this.camera = new TurretAim(targetTagIds);
    }

    /** The camera aimer underneath, for telemetry that already reads it. */
    public TurretAim camera() {
        return camera;
    }

    public void setTargetTagIds(int[] ids) {
        camera.setTargetTagIds(ids);
        forgetTrim();   // the old trim belonged to the old target
    }

    public void setAcceptAnyHive(boolean accept) {
        camera.setAcceptAnyHive(accept);
    }

    public void resetPhase() {
        camera.resetPhase();
        lead.reset();
        forgetTrim();
        // A fresh start gets the benefit of the doubt, the same as construction.
        poseTrusted = true;
        rejecting = false;
        rejectingSinceSec = Double.NaN;
    }

    /** Drop the camera correction. The aim falls back to raw odometry. */
    public void forgetTrim() {
        trimDeg = 0;
        trimEverSet = false;
    }

    /**
     * The angular disagreement between what the camera says and what the pose
     * says, wrapped to +/-180. Positive means the camera wants a bigger turret
     * angle than odometry does.
     *
     * Static and pure so it can be tested on a laptop. The camera is on the
     * TURRET, so its bearing is relative to where the turret currently points --
     * hence getAngle() + bearing, the same sum TurretAim itself commits.
     */
    public static double trimFrom(double turretAngleDeg, double smoothBearingDeg,
                                  double bearingSign, double aimBiasDeg,
                                  double odometryDesiredDeg) {
        double cameraAbsolute = turretAngleDeg + smoothBearingDeg * bearingSign + aimBiasDeg;
        return TurretGeometry.wrapDegrees(cameraAbsolute - odometryDesiredDeg);
    }

    /**
     * Call every loop.
     *
     * @param detections AprilTagProcessor.getDetections()
     * @param turret     the turret; only setTargetAngle() is ever called
     * @param robot      the pose. null falls back to camera-only aiming
     * @param targetX    field inches. NaN falls back to camera-only aiming
     * @param targetY    field inches
     * @param enabled    false = look and measure, but never command
     * @param timeSec    any steadily increasing clock, for the lead's rate
     */
    public void update(List<AprilTagDetection> detections, Turret turret,
                       Pose robot, double targetX, double targetY,
                       boolean enabled, double timeSec) {

        TurretGeometry geometry = TurretGeometry.forRobot();
        boolean canUseOdometry = USE_ODOMETRY_AIM
                && geometry.isConfigured()
                && robot != null
                && !Double.isNaN(targetX) && !Double.isNaN(targetY);

        if (!canUseOdometry) {
            // Camera owns the turret, exactly as it always did.
            camera.update(detections, turret, enabled);
            solution = null;
            onTarget = camera.isOnTarget();
            source = !USE_ODOMETRY_AIM ? "camera"
                    : !geometry.isConfigured() ? "camera (turret geometry not measured)"
                    : "camera (no pose or target)";
            status = camera.getStatus();
            return;
        }

        // Odometry owns the turret. The camera is handed enabled=false so it
        // MEASURES without commanding -- two things driving one turret would
        // fight, and TurretAim's step-then-go-blind cycle would undo the
        // continuous tracking every time it committed a move.
        // MONITORING SOLVE. Pure -- it commands nothing. It runs even after we
        // have fallen back to the camera, which is what lets a bad pose earn its
        // trust back instead of being written off until the next INIT.
        OdometryAim.Solution monitor = OdometryAim.solve(geometry, robot, targetX, targetY,
                turret.getAngle(), Turret.MIN_ANGLE, Turret.MAX_ANGLE, HYSTERESIS_DEG);

        // poseTrusted was decided by LAST loop's measurement, so the handover
        // costs one loop (~20 ms). Deciding it here instead would mean measuring
        // the camera before letting it command, and TurretAim.update() does both
        // in one call -- a loop of latency is the cheaper of the two.
        boolean odometryDrives = poseTrusted;

        // The camera only commands when odometry ISN'T. Two things driving one
        // turret would fight, and TurretAim's step-then-go-blind cycle would undo
        // the continuous tracking every time it committed a move.
        camera.update(detections, turret, enabled && !odometryDrives);

        updateTrim(turret, monitor, timeSec);

        if (!odometryDrives) {
            // FELL BACK. The pose disagreed with the camera by more than
            // MAX_TRIM_DEG for TRIM_DISTRUST_SEC, so it is not being used to aim.
            solution = null;
            onTarget = camera.isOnTarget();
            source = String.format("camera (POSE NOT TRUSTED, off by %.0f deg)",
                    Math.abs(lastRejectedDeg));
            status = camera.getStatus() + " | odometry rejected";
            return;
        }

        solution = monitor;
        lead.update(solution.desiredDeg, timeSec);

        double command = solution.commandDeg + appliedTrim() + lead.leadDeg();
        if (enabled && turret.isHomed()) {
            // setTargetAngle clips to the travel, so the trim and the lead
            // together cannot push the command outside it.
            turret.setTargetAngle(command);
        }

        onTarget = solution.reachable && turret.isOnTarget();
        source = !USE_CAMERA_TRIM ? "odometry (trim off, NO pose check)"
                : rejecting ? "odometry + STALE trim, camera disagrees"
                : trimEverSet ? "odometry + camera trim"
                : "odometry (no camera fix yet)";
        status = String.format("%s | %s | %.1f in",
                source,
                solution.reachable ? "in reach"
                        : String.format("OUT OF REACH by %.0f deg", solution.outOfReachDeg),
                solution.distance);
    }

    /**
     * Fold this frame's camera bearing into the trim, and decide whether the pose
     * is still worth aiming from.
     *
     * WHAT CHANGED 2026-11-12 AND WHY. The rejection below used to be a bare
     * {@code return}: a disagreement past MAX_TRIM_DEG was dropped as
     * "unbelievable" and nothing else happened. That reasoning only holds while
     * the pose is roughly right. When the pose is badly wrong the big
     * disagreement IS the truth, so the gate threw away the truth, the last
     * accepted trim kept being applied forever, and the turret aimed off a
     * drifted pose with no way to tell from the Driver Station. 6494 did exactly
     * that on the red hive after some hard driving -- trim frozen at 2.5 deg
     * while the aim was tens of degrees out, and showing the camera more tags
     * could not help because the measurements were being computed and binned.
     *
     * Now a sustained rejection means something: the pose loses its trust, the
     * stale trim is dropped, and the camera takes the turret back.
     */
    private void updateTrim(Turret turret, OdometryAim.Solution monitor, double timeSec) {
        if (!USE_CAMERA_TRIM) {
            // Trim off is opting out of the pose check too -- there is nothing to
            // compare the pose against. Odometry keeps the turret, right or wrong.
            rejecting = false;
            rejectingSinceSec = Double.NaN;
            return;
        }
        // No usable bearing this frame: can't judge the pose, so don't. Aiming
        // from odometry with no tags in sight is the whole point of odometry.
        if (!camera.hasFreshBearing()) return;
        if (camera.getAgeMs() > TRIM_MAX_AGE_MS) return;
        if (camera.getSecondsSinceSeen() > TRIM_FRESH_SEC) return;

        double measured = trimFrom(turret.getAngle(), camera.getSmoothBearing(),
                TurretAim.BEARING_SIGN, TurretAim.AIM_BIAS_DEG, monitor.desiredDeg);

        if (Math.abs(measured) > MAX_TRIM_DEG) {
            lastRejectedDeg = measured;
            rejecting = true;
            if (Double.isNaN(rejectingSinceSec)) rejectingSinceSec = timeSec;
            // Sustained, not a single bad frame -- one wild detection shouldn't
            // throw away a good pose.
            if (timeSec - rejectingSinceSec >= TRIM_DISTRUST_SEC) distrustPose();
            return;
        }

        rejecting = false;
        rejectingSinceSec = Double.NaN;
        // The camera and the pose agree again. Trust is earned back the same way
        // it was lost -- by measurement, not by a timer. Flapping is damped by
        // distrust needing TRIM_DISTRUST_SEC of disagreement while this needs one
        // good frame, so it leans toward using the camera.
        poseTrusted = true;
        // The first real fix lands whole, so the aim does not have to crawl to it
        // over a dozen frames. After that it is smoothed.
        trimDeg = trimEverSet ? trimDeg + TRIM_SMOOTHING * (measured - trimDeg) : measured;
        trimEverSet = true;
    }

    /** Hand the turret back to the camera and forget the trim built on a bad pose. */
    private void distrustPose() {
        if (!poseTrusted) return;
        poseTrusted = false;
        // This trim was measured against a pose we no longer believe. Keeping it
        // is the "stale forever" bug; dropping it means a re-trust starts clean.
        trimDeg = 0;
        trimEverSet = false;
        // The camera has been measuring with enabled=false, so its phase machine
        // is wherever it was left. Start a clean track rather than resuming.
        camera.resetPhase();
    }

    private double appliedTrim() {
        return USE_CAMERA_TRIM && trimEverSet ? trimDeg : 0;
    }

    // ---- Telemetry ----------------------------------------------------------

    /** One line for the Driver Station: what is aiming, whether it can, and how far. */
    public String getStatus() {
        return status;
    }

    /** "camera", "odometry", "odometry + camera trim", and so on. */
    public String getSource() {
        return source;
    }

    public boolean isOnTarget() {
        return onTarget;
    }

    /** True while the turret is being aimed from the pose rather than the camera. */
    public boolean isUsingOdometry() {
        return solution != null;
    }

    /** The camera correction being applied right now, degrees. 0 if there is none. */
    public double getTrimDeg() {
        return appliedTrim();
    }

    public boolean hasCameraFix() {
        return trimEverSet;
    }

    /**
     * False when odometry aiming has been dropped because the pose disagreed with
     * the camera. Op modes should surface this -- it is the difference between
     * "aiming by camera because you chose to" and "aiming by camera because the
     * pose went bad", and those want very different reactions.
     */
    public boolean isPoseTrusted() {
        return poseTrusted;
    }

    /** True while the camera is disagreeing with the pose by more than the cap. */
    public boolean isRejectingTrim() {
        return rejecting;
    }

    /** The last over-cap camera/pose disagreement, degrees. */
    public double getLastRejectedDeg() {
        return lastRejectedDeg;
    }

    /**
     * Flat distance from the turret axis to the target in inches, or NaN while
     * aiming from the camera. THIS is what ShotTable wants -- camera-only aiming
     * never produced a distance at all.
     */
    public double getDistance() {
        return solution == null ? Double.NaN : solution.distance;
    }

    /** True when the target is outside the turret's travel. Turn the robot. */
    public boolean isOutOfReach() {
        return solution == null ? camera.isOutOfReach() : !solution.reachable;
    }

    /** The last odometry solution, or null while aiming from the camera. */
    public OdometryAim.Solution getSolution() {
        return solution;
    }

    public double getLeadDeg() {
        return lead.leadDeg();
    }

    public double getTargetRateDegPerSec() {
        return lead.rateDegPerSec();
    }
}
