package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

/*
 * VisionPoseFilter: decides whether to TRUST a camera position fix, and how
 * much to nudge odometry toward it.
 *
 * Odometry (Pinpoint) is smooth but slowly drifts, and a bump from another
 * robot can knock it off. The camera doesn't drift, but it's slow, noisy, and
 * sometimes just wrong. So we let the camera CORRECT odometry, carefully:
 *
 *   1. check()  -- throw away fixes we shouldn't believe (design doc, "three rules")
 *   2. blend()  -- move the odometry pose a little bit toward a fix we do believe
 *
 * Why a little bit, not all the way? A single fix can be off by an inch or two.
 * Moving 10% of the way each time averages many fixes together, and the robot
 * never visibly "jumps." And NEVER feed this to follower.setPose() while
 * following a path -- the path jumps too. In auto, correct at a planned stop.
 *
 * Everything here is plain numbers, so it is tested on a laptop
 * (VisionPoseFilterTest). The numbers below are starting guesses.
 */
public class VisionPoseFilter {

    // ---- Trust rules (starting guesses -- tune on the robot in Phase 3) ----
    public static double MAX_ROBOT_SPEED = 2.0;       // in/s: faster than this, frame and pose disagree
    public static double MAX_ROBOT_TURN_RATE = 10.0;  // deg/s
    public static double MAX_FRAME_AGE_MS = 150;      // older frames describe where we WERE
    // TODO(measure, Phase 3): the distances where tag fixes are actually good.
    public static double MIN_TAG_RANGE = 12;          // in
    public static double MAX_TAG_RANGE = 84;          // in
    // A fix this far from odometry is more likely a bad frame than a real error.
    public static double MAX_JUMP = 18;               // in
    public static double MAX_HEADING_JUMP = 20;       // deg

    /** How far toward the fix each accepted frame moves us. 0.1 = 10% of the way. */
    public static double BLEND_FRACTION = 0.1;

    /** The answer from check(): ACCEPT, or the FIRST reason the fix was rejected. Show it in telemetry. */
    public enum Verdict {
        ACCEPT,
        TURRET_MOVING,
        ROBOT_MOVING,
        FRAME_TOO_OLD,
        TAG_OUT_OF_RANGE,
        TOO_FAR_FROM_ODOMETRY
    }

    // ========================================================================
    // STUDENT TODOs. Tests in VisionPoseFilterTest.
    // ========================================================================

    /**
     * STEP 7. Should we believe this fix? Check the rules IN THIS ORDER and
     * return the first one that fails, or ACCEPT if none do:
     *
     *   1. turretSettled is false                        -> TURRET_MOVING
     *   2. speed > MAX_ROBOT_SPEED, or |turnRate| > MAX_ROBOT_TURN_RATE
     *                                                    -> ROBOT_MOVING
     *   3. frameAgeMs > MAX_FRAME_AGE_MS                 -> FRAME_TOO_OLD
     *   4. tagRange outside MIN_TAG_RANGE..MAX_TAG_RANGE -> TAG_OUT_OF_RANGE
     *   5. vision is more than MAX_JUMP inches from odometry, OR its heading is
     *      more than MAX_HEADING_JUMP degrees off       -> TOO_FAR_FROM_ODOMETRY
     *
     * Careful in rule 5: headings of 359 deg and 1 deg are only 2 deg apart.
     * (TurretGeometry.wrapDegrees helps.)
     *
     * @param speed    robot speed, in/s       @param turnRate  deg/s, either direction
     */
    public static Verdict check(boolean turretSettled, double speed, double turnRate,
                                double frameAgeMs, double tagRange,
                                Pose odometry, Pose vision) {
        if (!turretSettled) return Verdict.TURRET_MOVING;
        if (speed > MAX_ROBOT_SPEED || Math.abs(turnRate) > MAX_ROBOT_TURN_RATE) return Verdict.ROBOT_MOVING;
        if (frameAgeMs > MAX_FRAME_AGE_MS) return Verdict.FRAME_TOO_OLD;
        if (tagRange < MIN_TAG_RANGE || tagRange > MAX_TAG_RANGE) return Verdict.TAG_OUT_OF_RANGE;
        double jump = Math.hypot(vision.getX() - odometry.getX(), vision.getY() - odometry.getY());
        double headingJump = TurretGeometry.wrapDegrees(Math.toDegrees(vision.getHeading() - odometry.getHeading()));
        if (jump > MAX_JUMP || Math.abs(headingJump) > MAX_HEADING_JUMP) return Verdict.TOO_FAR_FROM_ODOMETRY;
        return Verdict.ACCEPT;
    }

    /**
     * STEP 8. Move `fraction` of the way from odometry toward vision.
     * fraction 0 = stay at odometry, 1 = jump all the way to vision.
     *
     * x and y are easy. Heading is the trap: halfway between 350 deg and
     * 10 deg is 0 deg, NOT 180. Blend the SHORT way around the circle.
     */
    public static Pose blend(Pose odometry, Pose vision, double fraction) {
        double x = odometry.getX() + fraction * (vision.getX() - odometry.getX());
        double y = odometry.getY() + fraction * (vision.getY() - odometry.getY());
        double dh = Math.toRadians(TurretGeometry.wrapDegrees(Math.toDegrees(vision.getHeading() - odometry.getHeading())));
        return new Pose(x, y, odometry.getHeading() + fraction * dh);
    }
}
