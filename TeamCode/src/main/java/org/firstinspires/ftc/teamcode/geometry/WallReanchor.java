package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.field.HiveTags;
import org.firstinspires.ftc.teamcode.field.Zone;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Should we accept a wall re-anchor RIGHT NOW? The guard layer between
 * {@link WallFix} (which does the geometry) and the op mode (which owns the
 * button, the rumble and the pose).
 *
 * WHY THIS IS SEPARATE FROM WallFix
 * WallFix answers "where does the camera say we are, given we are square on this
 * wall". It has no idea whether the robot is actually against the wall, whether
 * it is still rolling, or whether the answer it produced is somewhere a robot
 * can physically be. Those are the questions that decide whether we WRITE the
 * pose, and writing the pose wrong is worse than not writing it at all -- it
 * hands the turret a confident lie, which is exactly the failure mode that was
 * breaking matches on 2026-11-12 (gamepad 1 X overwriting the heading).
 *
 * WHY IT IS A PURE FUNCTION
 * Same reason as VisionPoseFilter: every rule here is a laptop test. The robot
 * is a slow, scarce way to find out that a boolean was backwards.
 *
 * THE RULES, in order. First failure wins, because the first failure is the most
 * informative thing to show the driver.
 *
 *   1. the fix itself was refused            -> NO_FIX        (WallFix said why)
 *   2. robot still moving                    -> ROBOT_MOVING
 *   3. turret still slewing                  -> TURRET_MOVING
 *   4. the answer is off the field            -> OFF_FIELD
 *   5. the answer is inside a flower          -> IN_FLOWER
 *   6. the along-wall tags disagree           -> TAGS_DISAGREE
 *   7. the sight line is too oblique          -> TOO_OBLIQUE
 *   8. the heading snap was large             -> SNAP_TOO_BIG
 *   9. otherwise                              -> ACCEPT
 *
 * Rules 2 and 3 are about the MEASUREMENT being stale: the bearing was computed
 * against a turret angle and a pose from a frame that is already old, and a
 * moving robot invalidates the "square on the wall" premise anyway.
 *
 * Rules 4 and 5 are the cheap sanity net. A re-anchor that puts the robot inside
 * a flower or off the field is wrong no matter how good the bearings looked, and
 * it is the one class of error a driver cannot be expected to catch in a match.
 *
 * Rule 8 is deliberately NOT a hard refusal of a large snap on its own: WallFix
 * already refuses past MAX_SNAP_DEG. This is a second, tighter gate for WRITING
 * the pose, because a big snap means the odometry heading had drifted a long way
 * and heading is the one thing the wall gives us "for free" -- if that was badly
 * off, the along-wall answer was computed from a heading we should not trust.
 */
public final class WallReanchor {

    private WallReanchor() {}

    public enum Verdict {
        ACCEPT,
        NO_FIX,
        ROBOT_MOVING,
        TURRET_MOVING,
        OFF_FIELD,
        IN_FLOWER,
        TAGS_DISAGREE,
        TOO_OBLIQUE,
        SNAP_TOO_BIG;

        /** True for the one verdict that writes the pose. */
        public boolean accepted() {
            return this == ACCEPT;
        }
    }

    /**
     * Inches per second, and degrees per second, below which the robot counts as
     * parked. Not zero: a stopped robot still reports a little noise, and
     * demanding exact zero would mean the button never works.
     */
    public static double MAX_SPEED = 2.0;
    public static double MAX_TURN_RATE_DEG = 5.0;

    /**
     * How far the along-wall answers from different tags may disagree before the
     * frame is judged to be arguing with itself.
     *
     * WallFix.Result.spread is the farthest-apart pair. With tags centred the
     * whole chain's noise floor is about 0.6 in (measured 2026-11-13), so 3 in is
     * several times the noise and still well under the ~6.5 in a single
     * mis-assigned tag row would produce.
     */
    public static double MAX_SPREAD = 3.0;

    /**
     * Refuse when one degree of bearing error costs more than this many inches
     * along the wall. WallFix already refuses sight lines under
     * MIN_SIGHT_ANGLE_DEG; this is the same idea expressed in the units that
     * matter -- how much the answer AMPLIFIES a bearing error. About 1 in/deg at
     * the wall starts, so 3 allows a fair range of geometry and still rejects the
     * near-parallel cases where the answer is mostly leverage.
     */
    public static double MAX_INCHES_PER_DEG = 3.0;

    /**
     * Heading snap, degrees, above which we decline to write. WallFix's own
     * MAX_SNAP_DEG (10) is the limit for COMPUTING an answer; this is the tighter
     * limit for TRUSTING one. A snap this big means the odometry heading had
     * drifted far enough that the premise is shaky.
     */
    public static double MAX_SNAP_DEG = 6.0;

    /** Keep a re-anchor at least this far inside the field walls, inches. */
    public static double FIELD_MARGIN = 2.0;

    /**
     * Build the sightings from a camera frame and ask WallFix where we are. The
     * robot-side assembly half; {@link #check} is the part worth unit-testing and
     * this is the part that needs an SDK frame, so they are kept separate.
     *
     * THE TILT COMES FROM THE CALLER, not from the detection. The usual source
     * (TagLocalizer.turretFixOf, which tries both tilts and keeps the level one)
     * needs detection.robotPose, and that is null on TurretCamera because its
     * processor has no camera pose set. HiveGeometry.tiltWithRaised(aimCell) is
     * the cheap way round it: the operator has already said which cell they are
     * shooting at, and you shoot at the raised one.
     *
     * @param detections   straight from AprilTagProcessor.getDetections(); null is fine
     * @param pose         current odometry pose -- only its HEADING is used, snapped
     *                     to the nearest 90, so a drifted position does not matter
     * @param tilt         how the hive is tipped
     * @param turretDeg    turret angle now
     * @param face         which face of the robot is against the wall
     * @param faceDistance pose origin to that face, inches
     */
    public static WallFix.Result solveFrom(List<AprilTagDetection> detections, Pose pose,
                                           Tilt tilt, double turretDeg,
                                           WallFix.Face face, double faceDistance) {
        List<WallFix.Sighting> sightings = new ArrayList<>();
        if (detections != null) {
            for (AprilTagDetection d : detections) {
                // ftcPose null = the SDK had no size for it, so no bearing either.
                // Not a hive tag = not something HiveTags can place.
                if (d.ftcPose == null || FieldAprilTags.Cell.forTag(d.id) == null) continue;
                Rigid3 tag = HiveTags.tagPose(d.id, tilt);
                sightings.add(new WallFix.Sighting(tag.x(), tag.y(),
                        WallFix.bearingDeg(d.ftcPose.x, d.ftcPose.y, d.ftcPose.z,
                                TagLocalizer.CAMERA_PITCH_DEG, TagLocalizer.CAMERA_YAW_DEG)));
            }
        }
        return WallFix.solve(pose.getHeading(), face, faceDistance, turretDeg,
                TurretGeometry.forRobot(), TagLocalizer.CAMERA_FORWARD,
                TagLocalizer.CAMERA_LEFT, sightings);
    }

    /**
     * @param fix          what WallFix.solve() returned (may be a refusal, or null)
     * @param speed        robot speed, inches/sec
     * @param turnRateDeg  robot turn rate, degrees/sec
     * @param turretSettled Turret.isOnTarget()
     */
    public static Verdict check(WallFix.Result fix, double speed, double turnRateDeg,
                                boolean turretSettled) {
        if (fix == null || !fix.ok()) return Verdict.NO_FIX;
        if (Math.abs(speed) > MAX_SPEED) return Verdict.ROBOT_MOVING;
        if (Math.abs(turnRateDeg) > MAX_TURN_RATE_DEG) return Verdict.TURRET_MOVING;
        if (!turretSettled) return Verdict.TURRET_MOVING;

        Pose p = fix.robot;
        if (!onField(p)) return Verdict.OFF_FIELD;
        for (Zone flower : FieldConstants.FLOWERS) {
            if (flower.contains(p)) return Verdict.IN_FLOWER;
        }

        if (fix.spread > MAX_SPREAD) return Verdict.TAGS_DISAGREE;
        if (fix.inchesPerDeg > MAX_INCHES_PER_DEG) return Verdict.TOO_OBLIQUE;
        if (Math.abs(fix.snapDeg) > MAX_SNAP_DEG) return Verdict.SNAP_TOO_BIG;
        return Verdict.ACCEPT;
    }

    /** Inside the field walls, with FIELD_MARGIN to spare. */
    static boolean onField(Pose p) {
        return FieldConstants.FIELD.expandedBy(-FIELD_MARGIN).contains(p);
    }

    /**
     * One line for the Driver Station. The verdict plus the number that caused it,
     * because "TOO_OBLIQUE" on its own does not tell a driver whether to nudge the
     * robot a little or give up on this wall.
     */
    public static String describe(Verdict verdict, WallFix.Result fix) {
        if (verdict == null) return "--";
        if (fix == null) return verdict + " (no fix)";
        switch (verdict) {
            case NO_FIX:
                return "NO_FIX: " + fix.why;
            case TAGS_DISAGREE:
                return String.format("TAGS_DISAGREE: spread %.1f in > %.1f", fix.spread, MAX_SPREAD);
            case TOO_OBLIQUE:
                return String.format("TOO_OBLIQUE: %.1f in/deg > %.1f",
                        fix.inchesPerDeg, MAX_INCHES_PER_DEG);
            case SNAP_TOO_BIG:
                return String.format("SNAP_TOO_BIG: %+.1f deg > %.1f", fix.snapDeg, MAX_SNAP_DEG);
            case ACCEPT:
                return String.format("ACCEPT: %d tags, spread %.1f in, snap %+.1f deg",
                        fix.tagsUsed, fix.spread, fix.snapDeg);
            default:
                return verdict.toString();
        }
    }
}
