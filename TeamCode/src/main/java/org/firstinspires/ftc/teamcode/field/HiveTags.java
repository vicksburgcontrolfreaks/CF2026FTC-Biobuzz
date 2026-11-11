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

    public static double PIVOT_Y = 72.0;          // Fig. 9-8 (Evan 10/27): pivot is mid-frame, frame is centered on the field
    // FROM THE MANUAL, cross-checked on a field. The frame is 49.46 in. wide and
    // centred, spanning x 47.27-96.73, with the two hives side by side and red on
    // the low-x side; centred in its half gives 47.27 + 49.46/4 = 59.6.
    //
    // Shane taped our field 2026-11-11: red wall to red hive 58 in., blue wall to
    // blue hive 60 in. Those two should be EQUAL, so the 2 in. spread is our
    // field's own build tolerance and the truth is near the 59 in. average. That
    // confirms the "centred in its half" assumption, which was the shaky part.
    // Keeping the manual-derived number on purpose: a competition field is built
    // to CAD, and feeding in our field's error would aim wrong everywhere else.
    //
    // Must stay equal to HiveGeometry.RED_AUDIENCE_OPENING_X -- same physical spot.
    public static double RED_HIVE_X = 59.6;
    // CONFIRMED ON THE HIVE, Shane 2026-11-11: the 18.8 in. cell spacing from the
    // manual text checks out with a tape, so this half-spacing is real. Confirming
    // it is what made CELL_DROP solvable below.
    public static double CELL_ARM = 9.4;
    // SOLVED FROM A MEASUREMENT, Shane 2026-11-11. Was a made-up 10.0, which put
    // every tag 11 in. from where it really is.
    //
    // He levelled off the pivot axis itself (just under 44 in., matching
    // HIVE_PIVOT_HEIGHT) and measured the tag relative to that level, with the
    // RED AUDIENCE cell raised: bottom edge +5.0, centre +5.75, top edge +6.5 in.
    //
    // With the audience cell up, tagPose() puts the sticker centre this far above
    // the pivot:  CELL_ARM * sin(tilt) - CELL_DROP * cos(tilt)
    //           = 9.4 * sin(30) - CELL_DROP * cos(30)
    //           = 4.7 - 0.866 * CELL_DROP
    // Setting that to 5.75 gives CELL_DROP = -1.21. Negative: with the hive LEVEL
    // the sticker sits about 1.2 in. ABOVE the pivot plane, not 10 in. below it.
    //
    // This is one equation in one unknown only because CELL_ARM is now confirmed
    // (Shane measured the 18.8 in. cell spacing on the hive, 2026-11-11). Before
    // that it was one measurement against two unknowns and deliberately unsolved.
    //
    // UNCERTAINTY about +/-0.4. The same tape numbers also date the tilt
    // independently -- a 3.25 in. tag rising 1.5 in. (6.5 - 5.0) means
    // asin(1.5/3.25) = 27.5 deg, not 30 -- and at 27.5 deg the drop solves to
    // -1.59 instead. HiveTagsTest pins the 5.75 so a change here has to keep it.
    public static double CELL_DROP = -1.2;        // bottom face, inches below the pivot (hive level)
    // MEASURED, Shane 2026-11-11: a raised cell sits 30 deg up from parallel to
    // the floor, so the two resting positions are 60 deg apart (+/-30 from level).
    public static double HIVE_SWING_DEG = 60.0;
    // Manual Fig. 9-15 (read by Evan 10/27). 3.25 in. tags; layout along the row:
    //   [38]      [39]   |   [40]      [41]
    //   -6.5     -2.75   0   +2.75     +6.5     in. from the sticker centerline
    public static double TAG_OUTER_OFFSET = 6.5;  // centerline to the center of an end tag (end to end = 13)
    public static double TAG_SPACING = 3.75;      // center to center of the two tags on the SAME side (0.5 in. gap)

    // Which end of the row each of a cell's tag IDs is toward (-1 left, +1 right)
    // and whether it is the outer one, in the order Cell.tagIds lists them.
    // Figure 9-17 (read by Evan 10/27): looking UP from underneath (the camera's
    // view), every cell's IDs run lowest to highest, left to right.
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
        return (cell == Cell.RED_SCORING || cell == Cell.RED_AUDIENCE) ? Alliance.RED : Alliance.BLUE;
    }

    private static boolean isScoring(Cell cell) {
        return cell == Cell.RED_SCORING || cell == Cell.BLUE_SCORING;
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
        double side = isScoring(cell) ? +1 : -1;   // far cell sits toward +y
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
