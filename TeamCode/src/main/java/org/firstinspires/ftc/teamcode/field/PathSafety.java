package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Checks a path BEFORE the robot drives it: "would the robot's body hit the
 * hive, a flower, or a wall anywhere along this path?"
 *
 * How it works: walk along the path about 1 inch at a time. At each spot, draw
 * the box the robot covers there (using its size from RobotConstants and the
 * way it's facing), and see if that box overlaps anything.
 *
 * It WARNS, it doesn't stop the robot. Sometimes you mean to be close to
 * something (like driving up to a flower to collect). For those, list the zone
 * at the end so it's skipped:
 *     PathSafety.check("To flower", toFlower, FieldConstants.FLOWER_FAR_WALL);
 *
 * Limits -- this does NOT know about:
 *   - game pieces or other robots
 *   - the robot drifting off the path (it checks the path, not the real robot)
 *   - how accurate the field numbers are (about +/- 1 in., see FieldConstants)
 * So "no warnings" means "the plan looks OK", not "it can't crash".
 */
public final class PathSafety {

    private PathSafety() {} // nobody should make a PathSafety object

    // Things the robot should never touch
    private static final Zone[] OBSTACLES = {
            FieldConstants.HIVE,
            FieldConstants.FLOWER_FAR_WALL,
            FieldConstants.FLOWER_BLUE_WALL,
            FieldConstants.FLOWER_AUDIENCE_WALL,
            FieldConstants.FLOWER_RED_WALL,
    };

    // Robots start pushed up against the wall, and the field numbers are only
    // good to about 1 in., so let the robot's box poke this far past a wall
    // before we complain.
    private static final double WALL_TOLERANCE = 1.0;

    // Check one path. Returns a list of warnings (empty = looks OK).
    // allowedNear = zones you MEAN to get close to on this path, so skip them.
    public static List<String> check(String pathName, Path path, Zone... allowedNear) {
        List<String> warnings = new ArrayList<>();
        List<Zone> allowed = Arrays.asList(allowedNear);
        List<Zone> alreadyWarned = new ArrayList<>(); // one warning per obstacle is enough
        boolean wallWarned = false;

        // About one check per inch of path, and at least 10
        int steps = Math.max(10, (int) Math.ceil(path.length()));

        for (int i = 0; i <= steps; i++) {
            double t = (double) i / steps; // 0 = start of path, 1 = end
            Pose point = path.getPoint(t);
            Zone robot = robotFootprint(point.getX(), point.getY(), path.getHeadingGoal(t));

            for (Zone obstacle : OBSTACLES) {
                if (robot.overlaps(obstacle)
                        && !allowed.contains(obstacle)
                        && !alreadyWarned.contains(obstacle)) {
                    warnings.add(String.format("%s hits %s near (%.0f, %.0f)",
                            pathName, obstacle.name, point.getX(), point.getY()));
                    alreadyWarned.add(obstacle);
                }
            }

            if (!wallWarned && !insideWalls(robot)) {
                warnings.add(String.format("%s goes into a wall near (%.0f, %.0f)",
                        pathName, point.getX(), point.getY()));
                wallWarned = true;
            }
        }
        return warnings;
    }

    // Same thing for a PathChain: check each path in the chain.
    public static List<String> check(String chainName, PathChain chain, Zone... allowedNear) {
        List<String> warnings = new ArrayList<>();
        for (int i = 0; i < chain.size(); i++) {
            warnings.addAll(check(chainName + " part " + (i + 1), chain.getPath(i), allowedNear));
        }
        return warnings;
    }

    // The box (lined up with the walls) that the robot covers when its center is
    // at (x, y) facing 'heading'. Facing 0 or 180, it's exactly the robot.
    // Turned at an angle, the box is a bit BIGGER than the robot (it has to fit
    // the tilted corners), so angled paths may warn a little early. Better a
    // warning too many than a crash.
    private static Zone robotFootprint(double x, double y, double heading) {
        double cos = Math.abs(Math.cos(heading));
        double sin = Math.abs(Math.sin(heading));
        double halfX = (RobotConstants.ROBOT_LENGTH * cos + RobotConstants.ROBOT_WIDTH * sin) / 2;
        double halfY = (RobotConstants.ROBOT_LENGTH * sin + RobotConstants.ROBOT_WIDTH * cos) / 2;
        return new Zone("Robot", x - halfX, x + halfX, y - halfY, y + halfY);
    }

    private static boolean insideWalls(Zone robot) {
        // The tiny extra is for computer rounding: sin(180 deg) comes out as
        // 0.0000000000000001, not 0, which makes the robot look a hair bigger.
        Zone field = FieldConstants.FIELD.expandedBy(WALL_TOLERANCE + 1e-6);
        return robot.minX >= field.minX && robot.maxX <= field.maxX
            && robot.minY >= field.minY && robot.maxY <= field.maxY;
    }
}
