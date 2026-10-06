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
 */
public final class FieldAprilTags {

    private FieldAprilTags() {} // nobody should make a FieldAprilTags object

    // Manual 9.9: every tag is 3.25 in. square (the black square part)
    public static final double TAG_SIZE = 3.25;

    // Which tags are on which hive cell
    public static final int[] RED_FAR_CELL       = {30, 31, 32, 33}; // red, far from the audience
    public static final int[] RED_AUDIENCE_CELL  = {34, 35, 36, 37}; // red, audience side
    public static final int[] BLUE_AUDIENCE_CELL = {38, 39, 40, 41}; // blue, audience side
    public static final int[] BLUE_FAR_CELL      = {42, 43, 44, 45}; // blue, far from the audience

    // Hand this to AprilTagProcessor.Builder.setTagLibrary(...)
    public static AprilTagLibrary getLibrary() {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        addCell(builder, RED_FAR_CELL, "Red far cell");
        addCell(builder, RED_AUDIENCE_CELL, "Red audience cell");
        addCell(builder, BLUE_AUDIENCE_CELL, "Blue audience cell");
        addCell(builder, BLUE_FAR_CELL, "Blue far cell");
        return builder.build();
    }

    private static void addCell(AprilTagLibrary.Builder builder, int[] ids, String cellName) {
        for (int id : ids) {
            builder.addTag(id, cellName + " #" + id, TAG_SIZE, DistanceUnit.INCH);
        }
    }
}
