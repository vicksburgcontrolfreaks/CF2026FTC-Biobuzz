package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

import java.util.Arrays;
import java.util.List;

/*
 * WallFix: "the robot is square against a wall" + the camera's BEARING to the
 * hive tags -> where the robot is.
 *
 * MENTOR-LEVEL. Pure math, no SDK types, tested on a laptop (WallFixTest).
 *
 * WHY THIS EXISTS (Shane, 2026-11-13)
 * Odometry aiming is fast and good until the pose drifts, and the drivers crash
 * into walls constantly anyway. A wall gives two of the three pose numbers for
 * free, mechanically, with no camera error at all:
 *
 *   HEADING  -- pushed flat against a wall, the robot is square to it. Odometry
 *               only has to be within 45 deg to say WHICH square; rounding to
 *               the nearest 90 does the rest. (MAX_SNAP_DEG refuses a bigger
 *               correction, because that means the robot is not square.)
 *   DISTANCE -- the touching face is against the wall, so the pose origin is a
 *               known distance from it.
 *
 * That leaves ONE number, the position ALONG the wall, and it comes from the
 * camera's horizontal BEARING to a tag -- the thing a camera measures best. It
 * needs no range, no camera height, no tag orientation, so it sidesteps the
 * errors that dog the full TagLocalizer solve: pitch-dependent range, the edge-of-
 * frame bias in the solved heading, and single-tag pose ambiguity (which flips
 * the tag's ORIENTATION, not the direction to its centre). One tag is enough.
 *
 * THE MATH
 * Robot = P0 + s * u: P0 is on the line the pose origin must lie on, u runs
 * along the wall. The camera sits at robot + o, where o depends only on the
 * (known) heading and turret angle. The line of sight leaves the camera in
 * field direction d and must pass through the tag T:
 *
 *     cross(d, T - P0 - o - s*u) = 0   =>   s = cross(d, T - P0 - o) / cross(d, u)
 *
 * cross(d, u) is the sine of the angle between the sight line and the wall. Near
 * zero (looking ALONG the wall) a tiny bearing error is a huge along-wall error,
 * so sightings under MIN_SIGHT_ANGLE_DEG are refused. The sensitivity is
 * range / sin(angle) inches per radian of bearing error; Result.inchesPerDeg
 * reports it so the screen shows how forgiving the geometry is.
 *
 * WHAT IT STILL TRUSTS
 * Any BEARING bias -- CAMERA_YAW_DEG (assumed 0), TURRET_ZERO_DEG, lens centre --
 * moves the answer along the wall by inchesPerDeg per degree. But the turret
 * aims with the same camera, so a re-anchor made this way aims as well as camera
 * aiming does from that spot, and then keeps odometry's speed after.
 */
public final class WallFix {

    private WallFix() {}

    /** Refuse if odometry's heading is further than this from square. */
    public static double MAX_SNAP_DEG = 10.0;

    /** Refuse a sight line closer than this to parallel with the wall. */
    public static double MIN_SIGHT_ANGLE_DEG = 20.0;

    /** Which face of the robot is against the wall. Angle is from the robot's forward, CCW. */
    public enum Face {
        FRONT(0), LEFT(90), BACK(180), RIGHT(-90);

        public final double deg;

        Face(double deg) {
            this.deg = deg;
        }
    }

    /** The four field walls. FieldConstants: (0,0) = red/audience corner. */
    public enum Wall { RED, BLUE, AUDIENCE, SCORING }

    /** One tag in view: where it is on the field (x, y) and its turret-frame bearing. */
    public static final class Sighting {
        public final double tagX, tagY;
        /** Horizontal, in the TURRET's frame, degrees, + = to the left. See bearingDeg(). */
        public final double bearingDeg;

        public Sighting(double tagX, double tagY, double bearingDeg) {
            this.tagX = tagX;
            this.tagY = tagY;
            this.bearingDeg = bearingDeg;
        }
    }

    public static final class Result {
        /** Null when refused -- see why. */
        public final Pose robot;
        public final String why;
        public final Wall wall;
        /** Heading correction the rounding applied, degrees. Big = odometry heading drifted. */
        public final double snapDeg;
        public final int tagsUsed;
        /** Farthest-apart along-wall answers from the tags used, inches. */
        public final double spread;
        /** Along-wall inches per degree of bearing error, averaged over the tags used. */
        public final double inchesPerDeg;

        Result(Pose robot, String why, Wall wall, double snapDeg, int tagsUsed,
               double spread, double inchesPerDeg) {
            this.robot = robot;
            this.why = why;
            this.wall = wall;
            this.snapDeg = snapDeg;
            this.tagsUsed = tagsUsed;
            this.spread = spread;
            this.inchesPerDeg = inchesPerDeg;
        }

        public boolean ok() {
            return robot != null;
        }

        /** True when the wall runs along field x, so the unknown is x. */
        public boolean alongX() {
            return wall == Wall.AUDIENCE || wall == Wall.SCORING;
        }
    }

    /**
     * Horizontal bearing in the TURRET's frame, degrees, + = left, from the SDK's
     * camera-frame ftcPose (x right, y out of the lens, z up). Undoes the camera's
     * upward pitch the same way TurretAim.bearingOf() does, then adds the camera's
     * yaw on the turret (+ = camera turned left).
     */
    public static double bearingDeg(double camX, double camY, double camZ,
                                    double pitchUpDeg, double yawLeftDeg) {
        double p = Math.toRadians(pitchUpDeg);
        double forward = camY * Math.cos(p) - camZ * Math.sin(p);
        double left = -camX;
        return Math.toDegrees(Math.atan2(left, forward)) + yawLeftDeg;
    }

    /**
     * @param odometryHeadingRad the pose's heading now -- only has to be within
     *                           MAX_SNAP_DEG of square
     * @param face               which face is on the wall
     * @param faceDistance       pose origin to that face, inches
     * @param turretDeg          turret angle when the frame was taken
     * @param geometry           turret offset, zero and sign
     * @param cameraForward      camera ahead of the turret axis, inches
     * @param cameraLeft         camera left of the turret axis, inches
     */
    public static Result solve(double odometryHeadingRad, Face face, double faceDistance,
                               double turretDeg, TurretGeometry geometry,
                               double cameraForward, double cameraLeft,
                               List<Sighting> sightings) {
        double headingDeg = Math.toDegrees(odometryHeadingRad);
        double squareDeg = 90.0 * Math.round(headingDeg / 90.0);
        double snapDeg = TurretGeometry.wrapDegrees(squareDeg - headingDeg);
        if (Math.abs(snapDeg) > MAX_SNAP_DEG) {
            return refuse(String.format("not square: odometry is %.0f deg off a wall", snapDeg),
                    null, snapDeg);
        }
        double h = Math.toRadians(squareDeg);

        // Which wall the touching face points at.
        long faceDir = Math.round(TurretGeometry.wrapDegrees(squareDeg + face.deg) / 90.0);
        double size = FieldConstants.FIELD_SIZE;
        Wall wall;
        double p0x, p0y, ux, uy;
        if (faceDir == 0) {                       // +x
            wall = Wall.BLUE;     p0x = size - faceDistance; p0y = 0; ux = 0; uy = 1;
        } else if (faceDir == 2 || faceDir == -2) {   // -x
            wall = Wall.RED;      p0x = faceDistance;        p0y = 0; ux = 0; uy = 1;
        } else if (faceDir == 1) {                // +y
            wall = Wall.SCORING;  p0x = 0; p0y = size - faceDistance; ux = 1; uy = 0;
        } else {                                  // -y
            wall = Wall.AUDIENCE; p0x = 0; p0y = faceDistance;        ux = 1; uy = 0;
        }

        // Camera position relative to the pose origin: turret offset, then the
        // camera's own offset along where the turret points. Both are fixed once
        // heading and turret angle are, which is what makes this a 1-D problem.
        Pose atOrigin = new Pose(0, 0, h);
        Pose turretAxis = geometry.turretPosition(atOrigin);
        double phi = geometry.turretFieldHeading(atOrigin, turretDeg);
        double ox = turretAxis.getX() + cameraForward * Math.cos(phi) - cameraLeft * Math.sin(phi);
        double oy = turretAxis.getY() + cameraForward * Math.sin(phi) + cameraLeft * Math.cos(phi);

        double minSin = Math.sin(Math.toRadians(MIN_SIGHT_ANGLE_DEG));
        double[] answers = new double[sightings.size()];
        int n = 0;
        int alongWall = 0, behind = 0;
        double sumInPerDeg = 0;
        for (Sighting t : sightings) {
            double theta = phi + Math.toRadians(t.bearingDeg);
            double dx = Math.cos(theta), dy = Math.sin(theta);
            double crossDU = dx * uy - dy * ux;
            if (Math.abs(crossDU) < minSin) {
                alongWall++;
                continue;
            }
            double wx = t.tagX - p0x - ox;
            double wy = t.tagY - p0y - oy;
            double s = (dx * wy - dy * wx) / crossDU;
            // Camera -> tag must point the way the camera looks. If it doesn't, the
            // intersection is behind the lens and the bearing is nonsense.
            double tx = wx - s * ux, ty = wy - s * uy;
            if (dx * tx + dy * ty <= 0) {
                behind++;
                continue;
            }
            answers[n++] = s;
            sumInPerDeg += Math.hypot(tx, ty) / Math.abs(crossDU) * Math.PI / 180.0;
        }
        if (n == 0) {
            String why = sightings.isEmpty() ? "no tags"
                    : alongWall > 0 ? "sight line too close to along the wall"
                    : behind > 0 ? "bearing points away from the tag" : "no usable tag";
            return refuse(why, wall, snapDeg);
        }

        double[] used = Arrays.copyOf(answers, n);
        Arrays.sort(used);
        double s = n % 2 == 1 ? used[n / 2] : (used[n / 2 - 1] + used[n / 2]) / 2.0;
        Pose robot = new Pose(p0x + s * ux, p0y + s * uy, h);
        return new Result(robot, "ok", wall, snapDeg, n, used[n - 1] - used[0], sumInPerDeg / n);
    }

    private static Result refuse(String why, Wall wall, double snapDeg) {
        return new Result(null, why, wall, snapDeg, 0, 0, Double.NaN);
    }
}
