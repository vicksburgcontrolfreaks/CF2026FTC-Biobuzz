package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.geometry.Rigid3;

/*
 * HiveTags: where every hive AprilTag is on the field, in 3D, for BOTH ways a
 * hive can be tipped. TagLocalizer needs this to turn a tag sighting into a
 * robot position.
 *
 * MENTOR-LEVEL. Students measure the numbers; the code is done.
 *
 * THE MODEL (manual 9.6.2, 9.9):
 *   - Each hive is a seesaw on a pivot axis that runs along field x, at
 *     y = PIVOT_Y, z = HIVE_PIVOT_HEIGHT. Far cell toward +y, audience cell toward -y.
 *   - Picture the hive LEVEL (both cells the same height). Each cell's bottom
 *     face is CELL_ARM in. from the pivot along y and CELL_DROP in. below it,
 *     facing straight down.
 *   - A real hive rests tipped by +-HIVE_SWING_DEG / 2 from level: FAR_UP
 *     raises the +y side. The tags ride along, so a tag's pose is the level
 *     pose rotated about the pivot axis.
 *   - Each cell's 4 tags are one sticker, in a single ROW (not 2 x 2), lowest
 *     ID on the left, two tags each side of the centerline, NOT evenly spaced
 *     (see TAG_OUTER_OFFSET), "bottom edge oriented towards the center of the
 *     FIELD" (Figures 9-15/9-16/9-17; row layout read by Evan 10/27).
 *
 * Coordinates: Pedro field inches, z = up from the tiles. Tag frames follow
 * the SDK: x = right, y = down (toward the sticker's bottom edge), z = INTO
 * the tag. A tag on the bottom of a cell faces down, so its z points UP.
 *
 * !!!! EVERY NUMBER MARKED MADE-UP IS A PLACEHOLDER (2026-10-24) !!!!
 * Replace them from the field CAD and Figures 9-15/9-16/9-17 before trusting
 * any position this produces. The laptop tests check the math, not these numbers.
 */
public final class HiveTags {

    private HiveTags() {}

    public static double PIVOT_Y = 72.0;          // MADE-UP: assumes the pivot is on the field's midline
    public static double RED_HIVE_X = 60.0;       // MADE-UP: red hive center along x; blue is mirrored
    public static double CELL_ARM = 9.4;          // MADE-UP: half of the manual's ~18.8 in. cell spacing
    public static double CELL_DROP = 10.0;        // MADE-UP: bottom face, inches below the pivot (hive level)
    public static double HIVE_SWING_DEG = 40.0;   // MADE-UP: degrees between the two resting positions
    // Manual Fig. 9-15 (read by Evan 10/27). 3.25 in. tags; layout along the row:
    //   [38]      [39]   |   [40]      [41]
    //   -6.5     -2.75   0   +2.75     +6.5     in. from the sticker centerline
    public static double TAG_OUTER_OFFSET = 6.5;  // centerline to the center of an end tag (end to end = 13)
    public static double TAG_SPACING = 3.75;      // center to center of the two tags on the SAME side (0.5 in. gap)

    // Which end of the row each of a cell's tag IDs is toward (-1 left, +1 right)
    // and whether it is the outer one, in the order Cell.tagIds lists them.
    // Figure 9-17 (read by Evan 10/27): 38, 39, 40, 41 left to right. Assumed the
    // figure shows the camera's view and every cell follows the same pattern.
    private static final int[] ROW_SIDE = {-1, -1, +1, +1};
    private static final boolean[] ROW_OUTER = {true, false, false, true};

    /** Inches right of the sticker centerline for the tag at this index in Cell.tagIds. */
    static double rowOffset(int index) {
        double fromCenter = ROW_OUTER[index] ? TAG_OUTER_OFFSET : TAG_OUTER_OFFSET - TAG_SPACING;
        return ROW_SIDE[index] * fromCenter;
    }

    /**
     * The tilt the SDK's tag library is built for (FieldAprilTags.getLocalizationLibrary).
     * Any fixed choice works -- TagLocalizer corrects to the real tilt.
     */
    public static final Tilt REFERENCE = Tilt.FAR_UP;

    public static Alliance allianceOf(Cell cell) {
        return (cell == Cell.RED_FAR || cell == Cell.RED_AUDIENCE) ? Alliance.RED : Alliance.BLUE;
    }

    private static boolean isFar(Cell cell) {
        return cell == Cell.RED_FAR || cell == Cell.BLUE_FAR;
    }

    /** How far a hive is rotated from level about +x, in radians. FAR_UP lifts the +y side. */
    public static double tiltRadians(Tilt tilt) {
        double half = Math.toRadians(HIVE_SWING_DEG / 2);
        return tilt == Tilt.FAR_UP ? half : -half;
    }

    /** Moves anything rigidly attached to a hive from tilt `from` to tilt `to`. */
    public static Rigid3 tiltChange(Tilt from, Tilt to) {
        return Rigid3.rotXAbout(PIVOT_Y, FieldConstants.HIVE_PIVOT_HEIGHT, tiltRadians(to) - tiltRadians(from));
    }

    /** A tag's 3D field pose when its hive is tipped `tilt`. Throws for a non-hive tag. */
    public static Rigid3 tagPose(int id, Tilt tilt) {
        Cell cell = Cell.forTag(id);
        if (cell == null) throw new IllegalArgumentException("Tag " + id + " is not a hive tag");

        double hiveX = allianceOf(cell) == Alliance.RED ? RED_HIVE_X : FieldConstants.FIELD_SIZE - RED_HIVE_X;
        double side = isFar(cell) ? +1 : -1;   // far cell sits toward +y
        double cx = hiveX;
        double cy = PIVOT_Y + side * CELL_ARM;
        double cz = FieldConstants.HIVE_PIVOT_HEIGHT - CELL_DROP;

        // Sticker frame on a LEVEL hive: z up (into the downward-facing tag),
        // y (the sticker's "down") toward the pivot, x = y cross z.
        double[] zAxis = {0, 0, 1};
        double[] yAxis = {0, -side, 0};
        double[] xAxis = {yAxis[1] * zAxis[2] - yAxis[2] * zAxis[1],
                          yAxis[2] * zAxis[0] - yAxis[0] * zAxis[2],
                          yAxis[0] * zAxis[1] - yAxis[1] * zAxis[0]};
        Rigid3 sticker = Rigid3.fromAxes(cx, cy, cz, xAxis, yAxis, zAxis);

        int index = indexInCell(cell, id);
        Rigid3 levelTag = sticker.times(Rigid3.translation(rowOffset(index), 0, 0));

        Rigid3 tipped = Rigid3.rotXAbout(PIVOT_Y, FieldConstants.HIVE_PIVOT_HEIGHT, tiltRadians(tilt));
        return tipped.times(levelTag);
    }

    private static int indexInCell(Cell cell, int id) {
        for (int i = 0; i < cell.tagIds.length; i++) {
            if (cell.tagIds[i] == id) return i;
        }
        throw new IllegalArgumentException("Tag " + id + " is not on " + cell);
    }
}
