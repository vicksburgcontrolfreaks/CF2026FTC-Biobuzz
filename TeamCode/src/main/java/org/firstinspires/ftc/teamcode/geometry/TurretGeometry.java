package org.firstinspires.ftc.teamcode.geometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/*
 * TurretGeometry: the math that connects "where is the robot" to "where does
 * the turret point." No hardware, no camera -- just numbers in, numbers out,
 * so it can be tested on a laptop (see src/test/.../TurretGeometryTest.java).
 *
 * It works in BOTH directions, and both directions use the same few numbers:
 *
 *   AIMING (robot pose -> turret angle)
 *     Pedro says where the robot is. Where is the turret on the field, and
 *     what turret angle points it at the hive?
 *
 *   LOCALIZATION (turret pose -> robot pose)
 *     The camera rides on the turret, so a camera fix tells us where the
 *     TURRET is. Undo the turret angle and the offset to get the ROBOT pose.
 *
 * If both directions are right, going one way and then back lands exactly where
 * you started. The tests check that, which catches a sign mistake in either one.
 *
 * ---------------------------------------------------------------------------
 * UNITS AND DIRECTIONS (read this before writing any code below)
 *
 *   Field:  Pedro coordinates, inches. heading in RADIANS, 0 = facing +x,
 *           counter-clockwise (seen from above) is positive.
 *   Robot:  +x = the robot's front, +y = the robot's left side.
 *   Turret: DEGREES, the same numbers Turret.getAngle() reports.
 *
 * Mixing radians and degrees is the #1 way to get this wrong. Pedro is radians,
 * the turret is degrees. Convert with Math.toRadians() / Math.toDegrees().
 * ---------------------------------------------------------------------------
 *
 * See the design doc "Turret, Camera & Shooting: Design Plan", Phase 1.
 *
 * @Configurable so the measurements can be typed into Panels on the robot.
 * forRobot() reads them fresh, so a live edit takes effect on the next loop.
 */
@Configurable
public class TurretGeometry {

    // ---- Robot measurements -------------------------------------------------
    // TODO(measure): from CAD, then check with a tape measure. NaN means
    // "not measured yet" -- isConfigured() is false until all four are real.

    /** Turret axis, inches FORWARD of the robot's center (negative = behind). */
    public static double TURRET_OFFSET_X = Double.NaN;
    /** Turret axis, inches LEFT of the robot's center (negative = right). */
    public static double TURRET_OFFSET_Y = Double.NaN;
    /**
     * Which way turret 0 deg points, in degrees counter-clockwise from the
     * robot's front. Turret on the back, facing backward at 0 deg = 180.
     */
    public static double TURRET_ZERO_DEG = 0.0;  // Evan: faces front, per the build team.
    /**
     * +1 if a BIGGER turret angle turns the turret counter-clockwise (seen from
     * above), -1 if clockwise. Check it on the robot: jog the turret, watch
     * Turret.getAngle() and which way it turns.
     */
    public static double TURRET_CCW_SIGN = 1.0;  // turret-bringup.md range test: ticks rise CCW, ENCODER_DIRECTION = 1.0

    /** The geometry of OUR robot, from the constants above. */
    public static TurretGeometry forRobot() {
        return new TurretGeometry(TURRET_OFFSET_X, TURRET_OFFSET_Y, TURRET_ZERO_DEG, TURRET_CCW_SIGN);
    }

    public final double offsetX;
    public final double offsetY;
    public final double zeroDeg;
    public final double ccwSign;

    // Tests make their own made-up robots with this, so they never depend on
    // whatever is in the constants above today.
    public TurretGeometry(double offsetX, double offsetY, double zeroDeg, double ccwSign) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.zeroDeg = zeroDeg;
        this.ccwSign = ccwSign;
    }

    /** False until every measurement is filled in. Don't aim with an unconfigured robot. */
    public boolean isConfigured() {
        return !Double.isNaN(offsetX) && !Double.isNaN(offsetY)
                && !Double.isNaN(zeroDeg) && (ccwSign == 1.0 || ccwSign == -1.0);
    }

    // ========================================================================
    // STUDENT TODOs, in this order. Each one has tests in TurretGeometryTest.
    // Run them with:  gradlew :TeamCode:testDebugUnitTest
    // ========================================================================

    /**
     * STEP 1. Turn any angle into the same direction between -180 (not
     * included) and +180 (included).
     *   190 -> -170,   -190 -> 170,   540 -> 180,   -180 -> 180,   45 -> 45
     */
    public static double wrapDegrees(double degrees) {
        double d = degrees % 360.0;
        if (d <= -180) d += 360;
        if (d > 180) d -= 360;
        return d;
    }

    /**
     * STEP 2. Where the turret's axis is on the field. Returns a Pose whose x,y
     * are the turret's field position; its heading is just the robot's heading.
     *
     * Hint: the offset is in ROBOT directions (forward, left). Rotate it by the
     * robot's heading to get FIELD directions, then add the robot's position.
     */
    public Pose turretPosition(Pose robot) {
        double h = robot.getHeading();
        double x = robot.getX() + offsetX * Math.cos(h) - offsetY * Math.sin(h);
        double y = robot.getY() + offsetX * Math.sin(h) + offsetY * Math.cos(h);
        return new Pose(x, y, h);
    }

    /**
     * STEP 3. The turret angle (degrees, wrapped) that points the turret at a
     * spot on the field. This is the aiming answer.
     *
     * Hint, in words: find the turret's field position (step 2). Find the FIELD
     * direction from there to the target (Math.atan2). Take away the robot's
     * heading and the turret's zero direction. Use ccwSign for which way is
     * positive. Wrap it (step 1).
     */
    public double aimAngle(Pose robot, double targetX, double targetY) {
        Pose t = turretPosition(robot);
        double field = Math.toDegrees(Math.atan2(targetY - t.getY(), targetX - t.getX()));
        return wrapDegrees(ccwSign * wrapDegrees(field - Math.toDegrees(robot.getHeading()) - zeroDeg));
    }

    /** STEP 4. Flat (x,y) distance in inches from the turret axis to a target. Feeds the shooter speed table. */
    public double distanceTo(Pose robot, double targetX, double targetY) {
        Pose t = turretPosition(robot);
        return Math.hypot(targetX - t.getX(), targetY - t.getY());
    }

    /**
     * STEP 5. The opposite of step 3: given the turret angle, which FIELD
     * direction is the turret pointing? Radians, any range is fine.
     */
    public double turretFieldHeading(Pose robot, double turretDeg) {
        return robot.getHeading() + Math.toRadians(zeroDeg + ccwSign * turretDeg);
    }

    /**
     * STEP 6. Localization: undo everything. The camera tells us where the
     * turret axis is (turretX, turretY) and which field direction it faces
     * (turretFieldHeadingRad); the encoder tells us turretDeg. Where is the robot?
     *
     * Hint: get the robot's heading first (undo step 5). THEN you can rotate
     * the offset and subtract it (undo step 2).
     */
    public Pose robotPoseFromTurret(double turretX, double turretY, double turretFieldHeadingRad, double turretDeg) {
        double h = turretFieldHeadingRad - Math.toRadians(zeroDeg + ccwSign * turretDeg);
        double x = turretX - (offsetX * Math.cos(h) - offsetY * Math.sin(h));
        double y = turretY - (offsetX * Math.sin(h) + offsetY * Math.cos(h));
        return new Pose(x, y, h);
    }
}
