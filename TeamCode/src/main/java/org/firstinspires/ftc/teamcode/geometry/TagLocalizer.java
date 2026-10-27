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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    // ---- Combining the tags in one frame (solveFrame) ------------------------
    // A fix farther than this from the group's median is dropped as a bad tag
    // (usually the single-tag "pose flip"). Starting guesses -- tune in Phase 3.
    public static double OUTLIER_DISTANCE = 3.0;     // in
    public static double OUTLIER_HEADING_DEG = 5.0;

    /** One camera position fix, ready for VisionPoseFilter. */
    public static class Fix {
        public final Pose robot;
        public final TurretFix turret;     // combined fix: the kept tag nearest the average
        public final int tagId;            // combined fix: that same tag
        public final double tagRange;      // in, for VisionPoseFilter's range rule (combined: the mean)
        public final long frameNanos;      // when the frame was captured
        public final int tagsUsed;         // 1 from solve(); how many survived in solveFrame()
        public final double spread;        // in, farthest kept fix from the average; 0 for one tag

        Fix(Pose robot, TurretFix turret, int tagId, double tagRange, long frameNanos) {
            this(robot, turret, tagId, tagRange, frameNanos, 1, 0.0);
        }

        Fix(Pose robot, TurretFix turret, int tagId, double tagRange, long frameNanos,
            int tagsUsed, double spread) {
            this.robot = robot;
            this.turret = turret;
            this.tagId = tagId;
            this.tagRange = tagRange;
            this.frameNanos = frameNanos;
            this.tagsUsed = tagsUsed;
            this.spread = spread;
        }
    }

    private final TurretGeometry turret;

    public TagLocalizer(TurretGeometry turret) {
        this.turret = turret;
    }

    /**
     * The robot's pose from one detection, or null if the SDK had no pose for
     * it (not a hive tag) or neither hive tilt comes out level and on the floor.
     * Prefer solveFrame(): a frame usually shows all 4 of a cell's tags.
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

    /**
     * One fix from every hive tag in a frame (pass getDetections() straight in),
     * or null if none solve or the tags can't agree. See combine().
     */
    public Fix solveFrame(List<AprilTagDetection> detections, double turretDegAtFrame) {
        List<Fix> fixes = new ArrayList<>();
        for (AprilTagDetection d : detections) {
            Fix f = solve(d, turretDegAtFrame);
            if (f != null) fixes.add(f);
        }
        return combine(fixes);
    }

    /**
     * Average several fixes of the same frame, after dropping outliers.
     *
     * Why not just keep the tag with the best decisionMargin? decisionMargin
     * says how cleanly the tag's ID decoded, not how accurate its POSE is. The
     * pose error that actually bites is a single tag "flipping" to a mirror-image
     * solution inches away. A median sees that one tag as the odd one out; a plain
     * average would be dragged by it; "best margin" might pick it.
     *
     *   1. Median x and y of all fixes. The fix nearest it is the reference.
     *   2. Drop fixes more than OUTLIER_DISTANCE from the median, or more than
     *      OUTLIER_HEADING_DEG from the reference's heading.
     *   3. Average what's left (heading the short way round the circle).
     *
     * Two tags that disagree leave the median halfway between, and both get
     * dropped: null. That's on purpose -- a frame that argues with itself
     * shouldn't move odometry.
     *
     * The 4 tags of one cell share the camera-mount and hive errors, so averaging
     * cuts corner-detection noise but NOT calibration error. Don't expect 2x.
     */
    static Fix combine(List<Fix> fixes) {
        if (fixes.isEmpty()) return null;
        if (fixes.size() == 1) return fixes.get(0);

        double[] xs = new double[fixes.size()];
        double[] ys = new double[fixes.size()];
        for (int i = 0; i < fixes.size(); i++) {
            xs[i] = fixes.get(i).robot.getX();
            ys[i] = fixes.get(i).robot.getY();
        }
        double medX = median(xs), medY = median(ys);

        Fix reference = fixes.get(0);
        for (Fix f : fixes) {
            if (distance(f.robot, medX, medY) < distance(reference.robot, medX, medY)) reference = f;
        }

        List<Fix> kept = new ArrayList<>();
        for (Fix f : fixes) {
            double headingOff = TurretGeometry.wrapDegrees(
                    Math.toDegrees(f.robot.getHeading() - reference.robot.getHeading()));
            if (distance(f.robot, medX, medY) <= OUTLIER_DISTANCE
                    && Math.abs(headingOff) <= OUTLIER_HEADING_DEG) {
                kept.add(f);
            }
        }
        if (kept.isEmpty()) return null;

        // Heading: average the offsets from the reference, so 359 and 1 average to 0.
        double sumX = 0, sumY = 0, sumHeadingOff = 0, sumRange = 0;
        for (Fix f : kept) {
            sumX += f.robot.getX();
            sumY += f.robot.getY();
            sumHeadingOff += TurretGeometry.wrapDegrees(
                    Math.toDegrees(f.robot.getHeading() - reference.robot.getHeading()));
            sumRange += f.tagRange;
        }
        int n = kept.size();
        Pose mean = new Pose(sumX / n, sumY / n,
                reference.robot.getHeading() + Math.toRadians(sumHeadingOff / n));

        double spread = 0;
        Fix nearest = kept.get(0);
        for (Fix f : kept) {
            double d = distance(f.robot, mean.getX(), mean.getY());
            spread = Math.max(spread, d);
            if (d < distance(nearest.robot, mean.getX(), mean.getY())) nearest = f;
        }
        return new Fix(mean, nearest.turret, nearest.tagId, sumRange / n, nearest.frameNanos, n, spread);
    }

    private static double distance(Pose p, double x, double y) {
        return Math.hypot(p.getX() - x, p.getY() - y);
    }

    private static double median(double[] values) {
        double[] v = values.clone();
        Arrays.sort(v);
        int mid = v.length / 2;
        return v.length % 2 == 1 ? v[mid] : (v[mid - 1] + v[mid]) / 2.0;
    }
}
