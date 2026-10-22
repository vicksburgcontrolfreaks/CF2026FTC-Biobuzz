package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.field.FieldConstants;

/*
 * Named places on the field that our autos drive to. Every route in Routes
 * is built from these, so a point is typed in ONCE and shared.
 *
 * All points are RED side. Blue gets them spun 180 degrees automatically
 * (Alliance.map), so never type in a blue point.
 *
 * Most numbers here are figured out FROM the robot size and the field, not
 * typed in. When the real robot is measured, change RobotConstants and every
 * point moves to match -- in every auto at once.
 *
 * Adding a point: give it a clear name, build it from FieldConstants /
 * RobotConstants where you can, and say in a comment why it's there.
 *
 * Facing 180 (toward the red wall), the robot's LENGTH runs along x and its
 * WIDTH runs along y.
 */
public final class Waypoints {

    private Waypoints() {} // nobody should make a Waypoints object

    private static final double FACING_RED_WALL = Math.toRadians(180);
    private static final double HALF_LENGTH = RobotConstants.ROBOT_LENGTH / 2; // center to front/back
    private static final double HALF_WIDTH  = RobotConstants.ROBOT_WIDTH / 2;  // center to sides

    // ---------------- Knobs you might tune ----------------

    // Where along the garden wall the robot starts. Must keep the robot on
    // the red half: x + HALF_LENGTH <= 72.
    public static final double GARDEN_WALL_START_X = 59;

    // Gaps (inches) between the robot's edge and things we drive past.
    public static final double HIVE_SIDE_CLEARANCE = 5;  // left of the hive
    public static final double HIVE_TOP_CLEARANCE  = 4;  // above the hive
    public static final double FLOWER_CLEARANCE    = 1;  // far wall start, next to the flower.
                                                         // Only ~2.7 in. of room -- see FAR_WALL_START.

    // ---------------- Lines the points sit on ----------------

    // Robot touching the garden wall (for red, the audience wall)
    private static final double ON_GARDEN_WALL_Y = HALF_WIDTH;
    // Robot touching the far wall (for red, the rear wall)
    private static final double ON_FAR_WALL_Y = FieldConstants.FIELD_SIZE - HALF_WIDTH;
    // Just left of the hive
    private static final double LEFT_OF_HIVE_X = FieldConstants.HIVE.minX - HALF_LENGTH - HIVE_SIDE_CLEARANCE;
    // Just above the hive
    private static final double ABOVE_HIVE_Y = FieldConstants.HIVE.maxY + HALF_WIDTH + HIVE_TOP_CLEARANCE;

    // ---------------- Start positions (rule G304) ----------------

    // Against the garden wall.
    public static final Pose GARDEN_WALL_START = new Pose(GARDEN_WALL_START_X, ON_GARDEN_WALL_Y, FACING_RED_WALL);

    // Against the far wall, just right of the far-wall flower.
    // Tight fit! x has to be at least 51.3 + HALF_LENGTH to miss the flower,
    // and at most 72 - HALF_LENGTH to stay on the red half. With an 18 in.
    // robot that's only about 2.7 in. of room. Check it on a real field.
    public static final Pose FAR_WALL_START = new Pose(
            FieldConstants.FLOWER_FAR_WALL.maxX + HALF_LENGTH + FLOWER_CLEARANCE,
            ON_FAR_WALL_Y,
            FACING_RED_WALL);

    // ---------------- Garden ----------------

    // Tucked into the red garden corner, touching both walls.
    public static final Pose GARDEN_CORNER = new Pose(HALF_LENGTH, ON_GARDEN_WALL_Y, FACING_RED_WALL);

    // On the garden wall, lined up with the left side of the hive. Coming
    // from here, the robot drives along the wall into the garden corner --
    // the way the collector picks up the garden pollen best.
    public static final Pose GARDEN_APPROACH = new Pose(LEFT_OF_HIVE_X, ON_GARDEN_WALL_Y, FACING_RED_WALL);

    // ---------------- Around the hive ----------------

    // y = 59 is just "somewhere beside the hive" -- a corner to turn at,
    // picked so the diagonal from the garden misses the red-wall flower.
    public static final Pose HIVE_LEFT_LOW  = new Pose(LEFT_OF_HIVE_X, 59, FACING_RED_WALL);
    public static final Pose HIVE_LEFT_HIGH = new Pose(LEFT_OF_HIVE_X, ABOVE_HIVE_Y, FACING_RED_WALL);

    // Above the hive, lined up with the far wall start so that leg is straight.
    public static final Pose HIVE_TOP = new Pose(FAR_WALL_START.getX(), ABOVE_HIVE_Y, FACING_RED_WALL);

    // ---------------- Parking (end of auto) ----------------
    //
    // PARK = robot at least PARTLY in the loading zone at the end of auto.
    // The zone is only 24 in. long, so two 18 in. robots can't both fit all
    // the way in. Each park spot puts PARK_OVERLAP inches of the robot in the
    // zone at one end, with the rest hanging out. Then we and our partner
    // can both park, one at each end, with a gap between us.
    //
    // LEAVE = not touching the wall at the end of auto, so we park
    // PARK_WALL_GAP inches off the wall to get both.

    public static final double PARK_OVERLAP  = 8; // inches of robot inside the zone
    public static final double PARK_WALL_GAP = 2; // inches between robot and wall

    private static final double PARK_X = HALF_LENGTH + PARK_WALL_GAP;

    // End of the loading zone nearer our garden.
    public static final Pose PARK_GARDEN_SIDE = new Pose(PARK_X,
            FieldConstants.RED_LOADING_ZONE.minY + PARK_OVERLAP - HALF_WIDTH, FACING_RED_WALL);
    // End of the loading zone nearer the far wall.
    public static final Pose PARK_FAR_SIDE = new Pose(PARK_X,
            FieldConstants.RED_LOADING_ZONE.maxY - PARK_OVERLAP + HALF_WIDTH, FACING_RED_WALL);

    // ---------------- Launch spots (PLACEHOLDERS) ----------------
    //
    // The launcher isn't built yet, so nobody knows where it shoots well
    // from. These are just sensible places along the route. Once it works,
    // move them to wherever it actually hits the cell.

    // A few inches off the garden wall from the start. (Not exactly AT the
    // start: a path with zero length can confuse the path follower.)
    public static final double LAUNCH_OFF_WALL = 6;
    public static final Pose LAUNCH_GARDEN_WALL = new Pose(GARDEN_WALL_START_X,
            ON_GARDEN_WALL_Y + LAUNCH_OFF_WALL, FACING_RED_WALL);

    // At the far wall start, next to the far-wall flower.
    public static final Pose LAUNCH_FAR_WALL = FAR_WALL_START;

    // ---------------- Leaving the far wall ----------------

    // At the far wall the robot is only ~1 in. from the flower, so ANY move
    // that heads toward the red wall right away swings the robot's corner
    // into the flower (it hit it on the robot, 2026-10-22). Drive straight
    // off the wall to here first, then go wherever.
    public static final double FAR_WALL_EXIT_DISTANCE = 12; // inches straight off the wall
    public static final Pose FAR_WALL_EXIT = new Pose(FAR_WALL_START.getX(),
            ON_FAR_WALL_Y - FAR_WALL_EXIT_DISTANCE, FACING_RED_WALL);
}
