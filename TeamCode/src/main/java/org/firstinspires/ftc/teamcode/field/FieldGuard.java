package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * Keeps a DRIVING robot out of the hive, the flowers, and the walls.
 * (PathSafety checks a path before driving; this checks every move while
 * driving.)
 *
 * The idea: pretend the robot is a circle big enough to cover its corners at
 * any angle. If that circle (plus a safety margin) is touching an obstacle,
 * look at the move the robot wants to make and REMOVE the part of it that
 * heads into the obstacle. The rest of the move is kept, so the robot slides
 * along the edge instead of getting stuck.
 *
 *    wanted move           obstacle edge            allowed move
 *        ↗                 ─────────────                →
 *   (up and right)        (up is blocked)         (just right)
 *
 * Turning is always allowed: a circle looks the same at every angle.
 *
 * IMPORTANT: this only knows where the robot THINKS it is (odometry). If the
 * odometry is off by 3 in., the guard is off by 3 in. That's what
 * SAFETY_MARGIN is for. It does NOT know about game pieces or other robots.
 */
public final class FieldGuard {

    private FieldGuard() {} // nobody should make a FieldGuard object

    // Extra room around obstacles, mostly for odometry error. The odometry
    // hasn't been checked with Pedro's Localization Test yet -- after it is,
    // this could shrink.
    public static final double SAFETY_MARGIN = 3.0; // inches

    // The robot can't stop instantly. Look this far ahead in time: at
    // 20 in/s, 0.25 s adds 5 in. of room. Faster = more room.
    public static final double STOPPING_SECONDS = 0.25;

    // The circle that covers the robot's corners at any angle: half the
    // diagonal. An 18 x 18 robot -> about 12.7 in.
    public static final double ROBOT_RADIUS =
            Math.hypot(RobotConstants.ROBOT_LENGTH, RobotConstants.ROBOT_WIDTH) / 2;

    // The walls, as big zones just OUTSIDE the field, so they work like any
    // other obstacle. (Way bigger than needed on purpose -- simpler that way.)
    private static final double BIG = 100;
    private static final double SIZE = FieldConstants.FIELD_SIZE;
    private static final Zone[] WALLS = {
            new Zone("Red wall",      -BIG, 0,          -BIG, SIZE + BIG),
            new Zone("Blue wall",     SIZE, SIZE + BIG, -BIG, SIZE + BIG),
            new Zone("Audience wall", -BIG, SIZE + BIG, -BIG, 0),
            new Zone("Far wall",      -BIG, SIZE + BIG, SIZE, SIZE + BIG),
    };

    // What check() hands back: the allowed move (on the field), and the name
    // of whatever blocked part of it (null if nothing did).
    public static class Result {
        public final double moveX, moveY;
        public final String blockedBy;

        Result(double moveX, double moveY, String blockedBy) {
            this.moveX = moveX;
            this.moveY = moveY;
            this.blockedBy = blockedBy;
        }
    }

    /*
     * robotX, robotY = where the robot's center is (field inches)
     * moveX, moveY   = the move it wants, as a direction on the FIELD
     *                  (+x toward the blue wall, +y away from the audience)
     * speed          = how fast it's going right now (inches per second)
     */
    public static Result check(double robotX, double robotY,
                               double moveX, double moveY, double speed) {
        double keepAway = ROBOT_RADIUS + SAFETY_MARGIN + speed * STOPPING_SECONDS;
        String blockedBy = null;

        // If the robot thinks its CENTER is inside an obstacle or past a wall,
        // the odometry is badly wrong. We can't tell which way is safe, so
        // don't move at all.
        for (Zone[] group : new Zone[][]{FieldConstants.OBSTACLES, WALLS}) {
            for (Zone zone : group) {
                if (zone.contains(robotX, robotY)) {
                    return new Result(0, 0, "INSIDE " + zone.name + "?! odometry is off");
                }
            }
        }

        // Twice through: in a corner (hive + wall), fixing the move for one
        // obstacle can point it back into the other.
        for (int pass = 0; pass < 2; pass++) {
            for (Zone[] group : new Zone[][]{FieldConstants.OBSTACLES, WALLS}) {
                for (Zone zone : group) {
                    // Closest point of the zone to the robot's center
                    double closestX = Math.max(zone.minX, Math.min(robotX, zone.maxX));
                    double closestY = Math.max(zone.minY, Math.min(robotY, zone.maxY));
                    double towardX = closestX - robotX;
                    double towardY = closestY - robotY;
                    double distance = Math.hypot(towardX, towardY);

                    if (distance >= keepAway) continue; // far enough, ignore it
                    if (distance < 1e-6) return new Result(0, 0, zone.name); // exactly on the edge

                    towardX /= distance; // make it length 1: just a direction
                    towardY /= distance;

                    // How much of the move goes toward the zone? (the "dot product")
                    double intoIt = moveX * towardX + moveY * towardY;
                    if (intoIt > 0) {
                        // Take that part out; keep the sideways part
                        moveX -= intoIt * towardX;
                        moveY -= intoIt * towardY;
                        blockedBy = zone.name;
                    }
                }
            }
        }
        return new Result(moveX, moveY, blockedBy);
    }
}
