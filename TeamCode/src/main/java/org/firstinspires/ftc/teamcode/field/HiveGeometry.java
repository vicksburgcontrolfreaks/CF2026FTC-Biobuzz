package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;

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
@Configurable   // so the opening estimates can be typed into Panels on the field
public class HiveGeometry {

    private HiveGeometry() {}

    /** Which of a hive's two cells is facing up. "Far" = the side away from the audience. */
    public enum Tilt {
        FAR_UP, AUDIENCE_UP;

        /** After a tip, the other cell is up. */
        public Tilt afterTip() {
            return this == FAR_UP ? AUDIENCE_UP : FAR_UP;
        }
    }

    // ---- The RED AUDIENCE cell's opening, while it is the UP cell -------------
    // ESTIMATES, NOT MEASUREMENTS. Every number here is derived from the manual
    // plus Shane's 2026-11-11 hive measurements, and each rests on an assumption
    // that has not been checked on a field. They are here, rather than NaN, so
    // the turret can aim at the hive AT ALL and we can see how close it lands.
    // Replace them with the plumb-line measurement in docs/hive-opening-measure.md
    // and set OPENING_MEASURED = true.
    //
    // @Configurable: type better values straight into Panels on the field instead
    // of redeploying once per guess.

    // X = 59.6. The manual's hive frame is 49.46 in. wide, centred on the field,
    // so it spans x 47.27 to 96.73 with the two hives side by side. If each hive
    // sits centred in its half, red (the low-x side -- FieldConstants puts the
    // red wall at x = 0) lands at 47.27 + 49.46/4. THE ASSUMPTION is "centred in
    // its half", which is exactly what Figures 9-8..9-11 would settle.
    // Worth knowing: this should equal HiveTags.RED_HIVE_X, a made-up 60.0 that
    // happens to sit right next to this -- so it may be where 60.0 came from.
    public static double RED_AUDIENCE_OPENING_X = 59.6;

    // Y = 55.0. MEASURED, Shane 2026-11-11: audience wall to the centre of the
    // opening's face, with the red audience cell raised.
    //
    // This replaces a derived 63.9 that was wrong by about 9 in., and the reason
    // is worth keeping. The AprilTags sit on the cell's BOTTOM face; the opening
    // is the OPPOSITE face, 12 in. away through the cell (manual 9.6.2). The old
    // estimate put the opening directly above the bottom face's centre, so it
    // ignored that depth entirely. With the cell raised 30 deg the 12 in. runs
    // along the face normal (0, -0.5, 0.866), which moves the opening 6 in.
    // toward the audience and 10.4 in. up -- about what the measurement found.
    //
    // Shane's sanity check was the giveaway: 63.9 sits only 8 in. from the field
    // centre, and the hive does not reach that far in toward the middle.
    public static double RED_AUDIENCE_OPENING_Y = 55.0;

    // Z = 57.5, and Z DOES NOT AFFECT AIMING -- aimAngle() and distanceTo() are
    // flat x/y. It feeds LaunchReach. Shane measured the bottom of the opening at
    // just under 54 in.; the opening is 14 in. tall per the manual, lying in a
    // face tilted 30 deg, so its centre is about 14/2 * sin(30) = 3.5 in. above
    // the bottom edge. THE ASSUMPTION is which of the opening's dimensions runs
    // up the tilted face. This is the softest number of the three.
    public static double RED_AUDIENCE_OPENING_Z = 57.5;

    /**
     * Flip to true when the three numbers above are real measurements rather than
     * the derived estimates they are today. Op modes show this so an aim that
     * lands off the opening is not mistaken for a turret-geometry bug.
     */
    public static boolean OPENING_MEASURED = false;

    /**
     * Enough to AIM at the hive: x and y only, which is all TurretGeometry
     * .aimAngle() and distanceTo() read. Separate from isMeasured() because
     * requiring z blocked aiming on a number that aiming never uses.
     */
    public static boolean isAimMeasured() {
        return !Double.isNaN(RED_AUDIENCE_OPENING_X)
                && !Double.isNaN(RED_AUDIENCE_OPENING_Y);
    }

    /** All three, including the height LaunchReach needs. */
    public static boolean isMeasured() {
        return isAimMeasured() && !Double.isNaN(RED_AUDIENCE_OPENING_Z);
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
