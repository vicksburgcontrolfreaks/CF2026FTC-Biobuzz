package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.field.HiveTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * TagLocalizer: one AprilTag detection -> where the ROBOT is on the field.
 *
 * MENTOR-LEVEL. This is the 3D part of the plan. Students own TurretGeometry
 * and VisionPoseFilter; this class glues the SDK's camera math to them.
 *
 * THE CHAIN (design doc, "Localization with a rotating camera"):
 *
 *   tag on field --(SDK)--> turret base on field --(tilt fix)--> --(TurretGeometry step 6)--> robot
 *
 * 1. configure() describes the camera to the SDK relative to the TURRET AXIS at
 *    floor level, not the robot center. So detection.robotPose is really the
 *    pose of the turret base: on the floor, level, facing where the turret points.
 *
 * 2. THE HIVES TIP. The SDK's tag library is fixed when it is built, so it
 *    holds every tag where it sits with the hives tipped HiveTags.REFERENCE.
 *    If the hive is really tipped the other way, the tag has rotated about the
 *    pivot axis -- and so has the turret pose the SDK worked out from it. Rotate
 *    it back by the same amount (HiveTags.tiltChange) and it's exact.
 *
 * 3. WE DON'T HAVE TO KNOW THE TILT. Try both. The turret base really is on the
 *    floor and level, so the right tilt gives z ~ 0 and no tip; the wrong one is
 *    tipped by the whole HIVE_SWING_DEG and lifted off the floor. Keep the level
 *    one. That also tells us how the hive is tipped -- the camera sees a TIP happen.
 *
 * 4. Turret base -> robot with TurretGeometry.robotPoseFromTurret (student
 *    step 6), using the turret angle when the frame was taken.
 *
 * 5. Hand the Fix to VisionPoseFilter.check(), then blend().
 *
 * Steps 2-3 are plain math in solveTurret(), tested on a laptop against a copy
 * of the SDK's own computeRobotPose (TagLocalizerTest). What the laptop can't
 * check: that the camera numbers below match the real robot. Phase 3's gate
 * (park on tape-measured spots, compare) is where that gets proven.
 *
 * !!!! The CAMERA_* numbers and everything in HiveTags are MADE-UP (2026-10-24). !!!!
 */
public class TagLocalizer {

    // ---- Camera on the turret, with the turret at 0 deg ---------------------
    // MADE-UP until measured from CAD. Relative to the turret axis at FLOOR level.
    public static double CAMERA_FORWARD = 4.0;   // in, along the turret's pointing direction
    public static double CAMERA_LEFT = 0.0;      // in
    public static double CAMERA_UP = 12.0;       // in, lens height above the tiles
    public static double CAMERA_PITCH_DEG = 30;  // tipped UP from level, to see the downward tags
    public static double CAMERA_YAW_DEG = 0;     // turned left of the turret's pointing direction

    // How far from level (deg) and off the floor (in) the turret base may come
    // out and still be believed. The wrong hive tilt misses by ~HIVE_SWING_DEG.
    public static double LEVEL_TOLERANCE_DEG = 8;
    public static double FLOOR_TOLERANCE = 4;

    /**
     * Where the camera sits on the turret, in the SDK's robot frame (X right,
     * Y = the turret's pointing direction, Z up). The SDK's camera "pitch" is
     * measured from straight UP, so -90 is level and -60 is tipped up 30 deg.
     */
    public static Rigid3 cameraOnTurret() {
        return Rigid3.fromCameraPose(-CAMERA_LEFT, CAMERA_FORWARD, CAMERA_UP,
                CAMERA_YAW_DEG, CAMERA_PITCH_DEG - 90, 0);
    }

    /** Adds the camera pose and the hive tag library to an AprilTag processor builder. */
    public static AprilTagProcessor.Builder configure(AprilTagProcessor.Builder builder) {
        return builder
                .setTagLibrary(FieldAprilTags.getLocalizationLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, -CAMERA_LEFT, CAMERA_FORWARD, CAMERA_UP, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, CAMERA_YAW_DEG, CAMERA_PITCH_DEG - 90, 0, 0));
    }

    /** Where the turret base is, after the hive-tilt fix. */
    public static class TurretFix {
        public final double x, y;            // Pedro field inches
        public final double headingRad;      // the field direction the turret points
        public final Tilt hiveTilt;          // the tilt that came out level
        public final double levelErrorDeg;   // how far from level it came out
        public final double floorError;      // how far off the floor, inches
        public final boolean believable;     // within LEVEL_TOLERANCE_DEG and FLOOR_TOLERANCE

        TurretFix(Rigid3 turret, Tilt hiveTilt) {
            double[] pointing = turret.axis(1);
            this.x = turret.x();
            this.y = turret.y();
            this.headingRad = Math.atan2(pointing[1], pointing[0]);
            this.hiveTilt = hiveTilt;
            this.levelErrorDeg = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, turret.get(2, 2)))));
            this.floorError = Math.abs(turret.z());
            this.believable = levelErrorDeg <= LEVEL_TOLERANCE_DEG && floorError <= FLOOR_TOLERANCE;
        }
    }

    /**
     * Steps 2-3: fix the SDK's turret pose for the hive tilt. `turretFromSdk`
     * is detection.robotPose as a Rigid3 (computed against HiveTags.REFERENCE).
     * Returns the more level of the two tilts; check `believable` before using it.
     */
    public static TurretFix solveTurret(Rigid3 turretFromSdk) {
        TurretFix best = null;
        for (Tilt tilt : Tilt.values()) {
            Rigid3 corrected = HiveTags.tiltChange(HiveTags.REFERENCE, tilt).times(turretFromSdk);
            TurretFix candidate = new TurretFix(corrected, tilt);
            if (best == null || candidate.levelErrorDeg < best.levelErrorDeg) best = candidate;
        }
        return best;
    }

    /** One camera position fix, ready for VisionPoseFilter. */
    public static class Fix {
        public final Pose robot;
        public final TurretFix turret;
        public final int tagId;
        public final double tagRange;      // in, for VisionPoseFilter's range rule
        public final long frameNanos;      // when the frame was captured

        Fix(Pose robot, TurretFix turret, int tagId, double tagRange, long frameNanos) {
            this.robot = robot;
            this.turret = turret;
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
     * The robot's pose from one detection, or null if the SDK had no pose for
     * it (not a hive tag) or neither hive tilt comes out level and on the floor.
     * A frame usually shows all 4 of a cell's tags: that's 4 fixes, so average
     * them or keep the one with the best decisionMargin.
     *
     * Needs TurretGeometry step 6 (a student TODO) before it returns anything.
     */
    public Fix solve(AprilTagDetection detection, double turretDegAtFrame) {
        if (detection.robotPose == null) return null;
        Position p = detection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles o = detection.robotPose.getOrientation();
        Rigid3 fromSdk = Rigid3.fromPose3D(p.x, p.y, p.z,
                o.getYaw(AngleUnit.DEGREES), o.getPitch(AngleUnit.DEGREES), o.getRoll(AngleUnit.DEGREES));

        TurretFix fix = solveTurret(fromSdk);
        if (!fix.believable) return null;

        Pose robot = turret.robotPoseFromTurret(fix.x, fix.y, fix.headingRad, turretDegAtFrame);
        return new Fix(robot, fix, detection.id, detection.ftcPose.range, detection.frameAcquisitionNanoTime);
    }
}
