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
    // MEASURED ON THE HIVE, Shane 2026-11-11: with the hives LEVEL, the AprilTag
    // centres sit 14 in. from the pivot along y. That is exactly what this
    // constant is -- tagPose() puts a tag's y at PIVOT_Y +/- CELL_ARM, because the
    // row offset only moves tags along x.
    //
    // THE MANUAL'S 18.8 IN. AGREES, once you know what each number measures.
    // "Approximately 18.8 in. apart" (9.6.2) spans the two cells' BACK FACES, so
    // each back face is 9.4 in. from the pivot. The tag cluster is not at the
    // centre of the bottom face: Shane measured its centre 4.75 in. out from the
    // back edge. 9.4 + 4.75 = 14.15, against the 14 he measured directly. Two
    // independent routes to the same number, 0.15 in. apart.
    //
    // (An earlier pass read 18.8 as the tag spacing, got CELL_ARM = 9.4, and then
    // wrongly declared the manual's number useless when the 14 turned up. It is
    // not useless -- it measures the back faces, which is a different point.)
    public static double CELL_ARM = 14.0;
    // SOLVED FROM A MEASUREMENT, Shane 2026-11-11. Was a made-up 10.0.
    //
    // He levelled off the pivot axis itself (just under 44 in., matching
    // HIVE_PIVOT_HEIGHT) and measured the tag against that level with the RED
    // AUDIENCE cell raised: bottom edge +5.0, centre +5.75, top edge +6.5 in.
    //
    // With the audience cell up, tagPose() puts the sticker centre this far above
    // the pivot:  CELL_ARM * sin(tilt) - CELL_DROP * cos(tilt)
    //           = 14 * sin(30) - CELL_DROP * cos(30)
    //           = 7 - 0.866 * CELL_DROP
    // Setting that to 5.75 gives CELL_DROP = 1.44: with the hive LEVEL the sticker
    // sits about 1.4 in. BELOW the pivot plane. (An earlier pass had CELL_ARM at
    // 9.4 and solved this to -1.2, which put the sticker ABOVE the pivot -- a
    // worse shape, and a sign the arm was wrong.)
    //
    // UNCERTAINTY about +/-0.6, which is the tilt's. The same tape numbers date
    // the tilt independently -- a 3.25 in. tag rising 1.5 in. (6.5 - 5.0) means
    // asin(1.5/3.25) = 27.5 deg against the 30 we use -- and at 27.5 deg this
    // solves to 0.81 instead. HiveTagsTest pins the 5.75 so a change to CELL_ARM
    // or HIVE_SWING_DEG has to bring this with it.
    //
    // CROSS-CHECK against the OPENING, which is measured separately in
    // HiveGeometry. This is worth re-running after touching anything here.
    //
    // The cell, as Shane describes it (2026-11-11): a box 20 in. wide along x,
    // 14 in. tall perpendicular to the arm, 12 in. deep along the arm. Back face
    // at the inner end, 9.4 in. from the pivot. OPENING ON THE OUTER END FACE,
    // 21.4 in. out, facing along the arm -- so tipping the cell up 30 deg points
    // it outward and 30 deg above horizontal. Tag cluster on the bottom face,
    // 4.75 in. out from the back edge, which is why it rides the LOWER half of
    // that face once the cell is tipped.
    //
    // So tag -> opening is NOT a face normal. It is 7.25 in. further out along
    // the arm (21.4 - 14.15) and 7 in. perpendicular to it (bottom face up to the
    // opening's mid-height): 10.1 in. at 46 deg to the arm. Rotate that through
    // the 30 deg tilt and the opening centre lands at y 56.2, z 59.4, against the
    // y 55, z 59 Shane taped. 1.2 in. and 0.4 in. -- from the manual's box
    // dimensions and his tape readings, with nothing fitted.
    //
    // AN EARLIER VERSION OF THIS COMMENT WAS WRONG and claimed the check was
    // "12 in. at 30 deg". That assumed the tag sat at the CENTRE of the bottom
    // face and the opening directly opposite through the cell, on the TOP face.
    // Both are false. It appeared to agree to 1.5 in. by luck.
    public static double CELL_DROP = 1.44;        // bottom face, inches below the pivot (hive level)
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
