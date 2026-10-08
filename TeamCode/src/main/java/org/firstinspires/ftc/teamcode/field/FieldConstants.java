package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/*
 * Where things are on the BIOBUZZ field. This is the ONE place these numbers
 * live -- other code should use FieldConstants.HIVE etc. instead of typing
 * the numbers again.
 *
 * Source: docs/BIOBUZZ_Competition_Manual_V1.pdf (Version TU01), Section 9.
 * Positions marked "from Figure 9-2" were measured off the field drawing,
 * scaled using the 24 in. tile seams. The manual says drawings are good to
 * about +/- 1 in. Check them on a real field before trusting them to the inch.
 *
 * COORDINATES (inches, Pedro Pathing style):
 *   Stand where the audience stands and look at the field.
 *   (0, 0)  = near-left corner (red alliance side, audience wall)
 *   +x      = toward the blue alliance wall
 *   +y      = away from the audience
 *   heading 0 = facing +x, counter-clockwise is positive
 *
 * The field is the same when spun 180 degrees around its center (72, 72),
 * so we type in the red side and compute the blue side with rotate180().
 */
public final class FieldConstants {

    private FieldConstants() {} // nobody should make a FieldConstants object

    // Manual 9.2: about 144 x 144 in., measured between the inside of the walls.
    // 36 tiles, each about 24 in. square.
    public static final double FIELD_SIZE = 144.0;
    public static final double TILE_SIZE = 24.0;
    public static final double CENTER = FIELD_SIZE / 2;

    public static final Zone FIELD = new Zone("Field", 0, FIELD_SIZE, 0, FIELD_SIZE);

    // ---------------- HIVE (center structure) ----------------

    // Manual 9.6.1 / Figure 9-8: frame base is 49.46 in. wide (along x) by
    // 38.95 in. deep (along y), centered on the field. This is just the legs.
    public static final Zone HIVE_FRAME_BASE = new Zone("Hive frame base",
            CENTER - 49.46 / 2, CENTER + 49.46 / 2,
            CENTER - 38.95 / 2, CENTER + 38.95 / 2);

    // The whole hive seen from above, including cells that stick out past the
    // frame in y (from Figure 9-2). USE THIS ONE for staying away from the hive.
    public static final Zone HIVE = new Zone("Hive", 47, 97, 46, 98);

    // Manual 9.6.1: the hives pivot 43.95 in. above the tiles.
    public static final double HIVE_PIVOT_HEIGHT = 43.95;

    // ---------------- FLOWERS (4, on the walls, shared by both alliances) ----------------

    // Centers from Figure 9-2, on tile seams 48 in. from a corner. The boxes are
    // about 6.6 in. wide x 6 in. out from the wall, measured off the drawing.
    public static final Zone FLOWER_FAR_WALL      = new Zone("Flower (far wall)",      44.7, 51.3, 138, 144);
    public static final Zone FLOWER_RED_WALL      = new Zone("Flower (red wall)",      0,    6,    44.7, 51.3);
    public static final Zone FLOWER_AUDIENCE_WALL = FLOWER_FAR_WALL.rotated180("Flower (audience wall)");
    public static final Zone FLOWER_BLUE_WALL     = FLOWER_RED_WALL.rotated180("Flower (blue wall)");

    public static final Zone[] FLOWERS = {
            FLOWER_FAR_WALL, FLOWER_BLUE_WALL, FLOWER_AUDIENCE_WALL, FLOWER_RED_WALL
    };

    // Everything a robot must never drive into. PathSafety (path checks) and
    // FieldGuard (live driving) both use this list, so they always agree.
    public static final Zone[] OBSTACLES = {
            HIVE, FLOWER_FAR_WALL, FLOWER_BLUE_WALL, FLOWER_AUDIENCE_WALL, FLOWER_RED_WALL
    };

    // Manual 9.7
    public static final double FLOWER_TOP_OPENING_HEIGHT = 21.5;    // top of flower, where POLLEN/NECTAR go in
    public static final double FLOWER_TOP_OPENING_DIAMETER = 4.0;
    public static final double FLOWER_RETRIEVAL_HEIGHT = 3.55;      // opening at the bottom, POLLEN comes out only

    // ---------------- LOADING ZONES ----------------

    // Manual 9.3 / Figure 9-3: about 23 in. wide x 11 in. deep, against the
    // alliance wall, between the tile seams at y = 96 and y = 120 (Figure 9-2).
    public static final Zone RED_LOADING_ZONE  = new Zone("Red loading zone", 0, 11, 96, 120);
    public static final Zone BLUE_LOADING_ZONE = RED_LOADING_ZONE.rotated180("Blue loading zone");

    // ---------------- GARDENS ----------------

    // Manual 9.3 / Figure 9-3: about 23 in. x 2 in. strip of tape along the wall,
    // in the corner. Red is the near-left corner (Figure 9-2).
    public static final Zone RED_GARDEN  = new Zone("Red garden", 0, 24, 0, 2);
    public static final Zone BLUE_GARDEN = RED_GARDEN.rotated180("Blue garden");

    // ---------------- ROBOT SIZE RULES ----------------

    // Manual R102 / R105. These are the LIMITS from the rules, not our robot's
    // actual size -- that belongs in a RobotConstants file.
    public static final double MAX_START_SIZE = 18.0;        // 18 in. cube at the start
    public static final double MAX_EXPANDED_WIDTH = 18.0;    // after the start: 18 x 24 in. footprint
    public static final double MAX_EXPANDED_LENGTH = 24.0;

    // ---------------- HELPERS ----------------

    // Turn a red-side pose into the matching blue-side pose (or blue into red):
    // spin it 180 degrees around the field center.
    public static Pose rotate180(Pose pose) {
        return new Pose(
                FIELD_SIZE - pose.getX(),
                FIELD_SIZE - pose.getY(),
                normalizeRadians(pose.getHeading() + Math.PI));
    }

    // Keep an angle between 0 and 2*pi so headings are easy to read in telemetry.
    private static double normalizeRadians(double angle) {
        double twoPi = 2 * Math.PI;
        return ((angle % twoPi) + twoPi) % twoPi;
    }
}
