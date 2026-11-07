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
            return tilt == Tilt.FAR_UP ? Cell.RED_SCORING : Cell.RED_AUDIENCE;
        }
        return tilt == Tilt.FAR_UP ? Cell.BLUE_SCORING : Cell.BLUE_AUDIENCE;
    }

    /**
     * Half-width (in.) of the band around the midline where sideCell() keeps its
     * current answer, so driving along y = 72 doesn't flip the target back and forth.
     */
    public static double SIDE_DEADBAND = 6.0;

    /**
     * Which of OUR two cells is on the robot's side of the field -- the one to
     * target even while it faces DOWN, because when the hive tips it'll be the
     * up cell and we're already aimed (Shane, 2026-11-06). The cells mirror
     * across y = 72 for both alliances: audience cell below, scoring cell above.
     *
     * @param current the cell targeted now (kept inside the dead band); null = none yet
     */
    public static Cell sideCell(Alliance alliance, double robotY, Cell current) {
        double mid = FieldConstants.CENTER;
        Cell audience = alliance == Alliance.RED ? Cell.RED_AUDIENCE : Cell.BLUE_AUDIENCE;
        Cell scoring = alliance == Alliance.RED ? Cell.RED_SCORING : Cell.BLUE_SCORING;
        if (robotY > mid + SIDE_DEADBAND) return scoring;
        if (robotY < mid - SIDE_DEADBAND) return audience;
        if (current == audience || current == scoring) return current;
        return robotY >= mid ? scoring : audience;
    }

    /**
     * "Our cells, camera picks": which of OUR two cells to aim at, from the tag
     * IDs the camera sees right now. For when the robot's position isn't known
     * (no auto pose), so sideCell() can't be used.
     *
     * STICKY: keeps the current cell while ANY of its tags is in view, and only
     * moves to our other cell once the current one is out of view and the other
     * is seen. With both in frame it doesn't flip back and forth (every switch
     * throws away TurretAim's averaged reading). The other alliance's tags are
     * ignored completely -- that's the whole difference from "any hive".
     *
     * @param current the cell targeted now; null or not ours = pick fresh
     * @return the cell to target (current if nothing of ours is in view)
     */
    public static Cell cameraCell(Alliance alliance, int[] visibleIds, Cell current) {
        Cell audience = alliance == Alliance.RED ? Cell.RED_AUDIENCE : Cell.BLUE_AUDIENCE;
        Cell scoring = alliance == Alliance.RED ? Cell.RED_SCORING : Cell.BLUE_SCORING;
        int seenAudience = 0, seenScoring = 0;
        for (int id : visibleIds) {
            if (audience.hasTag(id)) seenAudience++;
            if (scoring.hasTag(id)) seenScoring++;
        }
        boolean currentIsOurs = current == audience || current == scoring;
        if (currentIsOurs && (current == audience ? seenAudience : seenScoring) > 0) return current;
        if (seenAudience == 0 && seenScoring == 0) return currentIsOurs ? current : scoring;
        return seenAudience > seenScoring ? audience : scoring;
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
            case RED_SCORING:       return new double[] { redAudX,        size - redAudY, z };  // mirror in y
            case BLUE_SCORING:      return new double[] { size - redAudX, size - redAudY, z };  // red audience spun 180
            case BLUE_AUDIENCE: return new double[] { size - redAudX, redAudY,        z };  // red scoring spun 180
            default: throw new IllegalArgumentException("Unknown cell " + cell);
        }
    }
}
