package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * TagLocalizer: one AprilTag detection -> where the ROBOT is on the field.
 *
 * MENTOR-LEVEL. This is the 3D part of the plan. Students own TurretGeometry
 * and VisionPoseFilter; this class glues the SDK's camera math to them.
 *
 * THE CHAIN (design doc, "Localization with a rotating camera"):
 *
 *   tag on field --(SDK)--> turret base on field --(TurretGeometry step 6)--> robot
 *
 * 1. Describe the camera to the SDK relative to the TURRET AXIS at floor level,
 *    not the robot center: AprilTagProcessor.Builder.setCameraPose(position,
 *    orientation) with the CAMERA_* numbers below. Then detection.robotPose
 *    is really the pose of the turret base. (Confirmed in SDK 11.2.1 source.)
 *
 * 2. THE HIVES TIP. A tag's field pose depends on whether its cell is up or
 *    down, and the SDK's tag library is fixed when the processor is built.
 *    Plan: give the library each tag's pose for a REFERENCE tilt. When the
 *    real tilt differs, the tag has rotated about the hive pivot axis (a line
 *    parallel to field x at y = 72, z = 43.95 -- TODO confirm in CAD) by the
 *    tilt swing, so rotate the SDK's turret-base pose about that same axis by
 *    the same angle. Exact correction, no new camera math.
 *
 * 3. WE DON'T HAVE TO KNOW THE TILT. Solve both candidates. The turret base
 *    really is on the floor and level, so the right candidate has z ~ 0 and
 *    pitch/roll ~ 0; the wrong one is tipped by the whole hive swing. Keep the
 *    level one. That also tells us the hive state, which picks the aim cell
 *    (HiveGeometry.upCell) -- the camera notices a tip for free.
 *
 * 4. Pose3D yaw -> turret field heading; turret encoder angle AT FRAME TIME
 *    (detection.frameAcquisitionNanoTime; Phase 1-3 just require the turret to
 *    be settled) -> TurretGeometry.robotPoseFromTurret().
 *
 * 5. Hand the Fix to VisionPoseFilter.check(), then blend().
 *
 * CONVENTION RISK: the SDK's robot frame is X right, Y forward, Z up, with
 * yaw/pitch/roll in its own order -- not Pedro's. Verify every axis on the
 * robot with a tape measure before trusting a number (Phase 3 gate).
 */
public class TagLocalizer {

    // ---- Camera on the turret ----------------------------------------------
    // TODO(measure, CAD): camera lens position relative to the turret axis at
    // floor level, and its aim, with the turret at 0 deg.
    public static double CAMERA_FORWARD = Double.NaN;  // in, along turret 0 deg direction
    public static double CAMERA_LEFT = Double.NaN;     // in
    public static double CAMERA_UP = Double.NaN;       // in, lens height above the tiles
    public static double CAMERA_PITCH_DEG = 30;        // tipped up to see the downward tags -- check it
    public static double CAMERA_YAW_DEG = Double.NaN;  // vs turret 0 deg; should be ~0

    // ---- Hive tilt ----------------------------------------------------------
    // TODO(measure, CAD): how far a cell swings, in degrees, between its up and
    // down resting positions.
    public static double HIVE_SWING_DEG = Double.NaN;
    // How far off level (degrees) and off the floor (in) a candidate may be
    // and still count as "the right tilt." Tune on the robot.
    public static double LEVEL_TOLERANCE_DEG = 8;
    public static double FLOOR_TOLERANCE = 4;

    /** One camera position fix, ready for VisionPoseFilter. */
    public static class Fix {
        public final Pose robot;
        public final HiveGeometry.Tilt hiveTilt;   // which tilt made the fix level
        public final int tagId;
        public final double tagRange;              // in, for VisionPoseFilter's range rule
        public final long frameNanos;              // when the frame was captured

        public Fix(Pose robot, HiveGeometry.Tilt hiveTilt, int tagId, double tagRange, long frameNanos) {
            this.robot = robot;
            this.hiveTilt = hiveTilt;
            this.tagId = tagId;
            this.tagRange = tagRange;
            this.frameNanos = frameNanos;
        }
    }

    private final TurretGeometry turret;

    public TagLocalizer(TurretGeometry turret) {
        this.turret = turret;
    }

    /**
     * The robot's pose from one detection, or null if neither hive tilt gives a
     * level, on-the-floor answer (bad frame, wrong tag poses, or a wrong camera
     * measurement -- telemetry should say which candidate was closest and why).
     */
    public Fix solve(AprilTagDetection detection, double turretDegAtFrame) {
        // TODO(mentor): steps 2-4 above. Needs FieldAprilTags to carry each
        // tag's field pose (reference tilt) and the CAD numbers in this file.
        throw new UnsupportedOperationException("TODO(mentor): TagLocalizer.solve");
    }
}
