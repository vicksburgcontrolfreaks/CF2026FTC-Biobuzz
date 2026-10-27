package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;

/*
 * The hives TIP, and that changes where we aim.
 *
 * Manual 9.6.2: each alliance has one HIVE with two CELLS, one toward the far
 * wall and one toward the audience, about 18.8 in. apart, on a pivot 43.95 in.
 * up. It's a seesaw with two resting positions: one cell faces UP (that's the
 * one we launch into), the other faces down. Fill the up cell enough and the
 * hive tips -- now the OTHER cell is up, and the turret has to aim there.
 *
 * So "aim at our hive" really means "aim at the opening of whichever of our
 * two cells is up right now." This file answers that.
 *
 * SYMMETRY: we measure ONE cell and get the other three for free.
 *   - Our two cells mirror each other across the middle of the field (y = 72).
 *     The pivot is at y = 72: Fig. 9-8 shows it mid-frame and the frame is
 *     centered on the field (Evan, 10/27; manual drawings are +-1 in.).
 *   - Blue is red spun 180 deg around the field center, like FieldConstants.
 *     Spinning swaps far and audience: red FAR <-> blue AUDIENCE.
 *
 * Coordinates are Pedro field inches, same as FieldConstants.
 */
public final class HiveGeometry {

    private HiveGeometry() {}

    /** Which of a hive's two cells is facing up. "Far" = the side away from the audience. */
    public enum Tilt {
        FAR_UP, AUDIENCE_UP;

        /** After a tip, the other cell is up. */
        public Tilt afterTip() {
            return this == FAR_UP ? AUDIENCE_UP : FAR_UP;
        }
    }

    // ---- Measured: the RED AUDIENCE cell's opening, while it is the UP cell ----
    // TODO(measure in CAD): the center of the opening we launch into, in field
    // inches. The FIRST field CAD (Playing Field Resources page) has the hive in
    // one resting position -- if it shows the other cell up, measure that cell
    // and mirror it (y -> 144 - y) to get this one. NaN = not measured yet.
    public static double RED_AUDIENCE_OPENING_X = Double.NaN;
    public static double RED_AUDIENCE_OPENING_Y = Double.NaN;
    public static double RED_AUDIENCE_OPENING_Z = Double.NaN;   // height above the tiles

    /** False until the opening is measured. */
    public static boolean isMeasured() {
        return !Double.isNaN(RED_AUDIENCE_OPENING_X)
                && !Double.isNaN(RED_AUDIENCE_OPENING_Y)
                && !Double.isNaN(RED_AUDIENCE_OPENING_Z);
    }

    /** Which cell an alliance launches into, given how its hive is tipped. */
    public static Cell upCell(Alliance alliance, Tilt tilt) {
        if (alliance == Alliance.RED) {
            return tilt == Tilt.FAR_UP ? Cell.RED_FAR : Cell.RED_AUDIENCE;
        }
        return tilt == Tilt.FAR_UP ? Cell.BLUE_FAR : Cell.BLUE_AUDIENCE;
    }

    /** {x, y, z}: where to aim for this alliance's up cell. Feed x, y to TurretGeometry.aimAngle(). */
    public static double[] aimPoint(Alliance alliance, Tilt tilt) {
        return openingOf(upCell(alliance, tilt),
                RED_AUDIENCE_OPENING_X, RED_AUDIENCE_OPENING_Y, RED_AUDIENCE_OPENING_Z);
    }

    // Separate from aimPoint() so tests can check the symmetry with made-up numbers.
    static double[] openingOf(Cell cell, double redAudX, double redAudY, double z) {
        double size = FieldConstants.FIELD_SIZE;
        switch (cell) {
            case RED_AUDIENCE:  return new double[] { redAudX,        redAudY,        z };
            case RED_FAR:       return new double[] { redAudX,        size - redAudY, z };  // mirror in y
            case BLUE_FAR:      return new double[] { size - redAudX, size - redAudY, z };  // red audience spun 180
            case BLUE_AUDIENCE: return new double[] { size - redAudX, redAudY,        z };  // red far spun 180
            default: throw new IllegalArgumentException("Unknown cell " + cell);
        }
    }
}
