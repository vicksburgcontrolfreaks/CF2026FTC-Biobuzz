package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

/*
 * The AprilTags on the BIOBUZZ field.
 *
 * Source: docs/BIOBUZZ_Competition_Manual_V1.pdf, Section 9.9 (Figure 9-17).
 * Each hive CELL has a cluster of 4 tags on its BOTTOM, facing DOWN toward
 * the tiles. So to see them, the camera has to look UP at the hive.
 *
 * Why we need this file: the FTC SDK we use (11.2.1) only knows tags from past
 * games. It can still SEE a BIOBUZZ tag and tell us its ID, but it can't say
 * how far away the tag is unless it knows how big the tag is. getLibrary()
 * tells it.
 *
 * This is the ONE place the tag-ID-to-hive-cell mapping lives. If you find
 * yourself typing {38, 39, 40, 41} anywhere else, use Cell instead -- a second
 * copy of these numbers is a second thing to get wrong when the manual changes.
 */
public final class FieldAprilTags {

    private FieldAprilTags() {} // nobody should make a FieldAprilTags object

    // Manual 9.9: every tag is 3.25 in. square (the black square part)
    public static final double TAG_SIZE = 3.25;

    /**
     * The four hive cells and the tag cluster on each one.
     *
     * Use Cell.forTag(id) to go from a tag the camera found to the cell it
     * belongs to -- that is how an op mode can say "you are looking at the Blue
     * audience hive" instead of making someone memorise which IDs are which.
     */
    public enum Cell {
        RED_FAR("Red far", 30, 31, 32, 33),
        RED_AUDIENCE("Red audience", 34, 35, 36, 37),
        BLUE_AUDIENCE("Blue audience", 38, 39, 40, 41),
        BLUE_FAR("Blue far", 42, 43, 44, 45);

        /** For telemetry -- "Blue audience", not "BLUE_AUDIENCE". */
        public final String displayName;
        /** The four tag IDs on this cell's underside. */
        public final int[] tagIds;

        Cell(String displayName, int... tagIds) {
            this.displayName = displayName;
            this.tagIds = tagIds;
        }

        public boolean hasTag(int id) {
            for (int tagId : tagIds) {
                if (tagId == id) return true;
            }
            return false;
        }

        /** Which cell a tag belongs to, or null if it is not a BIOBUZZ hive tag. */
        public static Cell forTag(int id) {
            for (Cell cell : values()) {
                if (cell.hasTag(id)) return cell;
            }
            return null;
        }

        @Override
        public String toString() {
            return displayName;
        }
    }

    // Kept so existing code reads the same. These are the SAME arrays the enum
    // holds, not copies -- edit the enum, not these.
    public static final int[] RED_FAR_CELL       = Cell.RED_FAR.tagIds;
    public static final int[] RED_AUDIENCE_CELL  = Cell.RED_AUDIENCE.tagIds;
    public static final int[] BLUE_AUDIENCE_CELL = Cell.BLUE_AUDIENCE.tagIds;
    public static final int[] BLUE_FAR_CELL      = Cell.BLUE_FAR.tagIds;

    // Hand this to AprilTagProcessor.Builder.setTagLibrary(...)
    public static AprilTagLibrary getLibrary() {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (Cell cell : Cell.values()) {
            for (int id : cell.tagIds) {
                builder.addTag(id, cell.displayName + " cell #" + id, TAG_SIZE, DistanceUnit.INCH);
            }
        }
        return builder.build();
    }
}
