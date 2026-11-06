package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

public class HiveGeometryTest {

    private static final double EPS = 1e-9;

    @Test public void eachAllianceLaunchesIntoItsOwnUpCell() {
        assertEquals(Cell.RED_SCORING,       HiveGeometry.upCell(Alliance.RED,  Tilt.FAR_UP));
        assertEquals(Cell.RED_AUDIENCE,  HiveGeometry.upCell(Alliance.RED,  Tilt.AUDIENCE_UP));
        assertEquals(Cell.BLUE_SCORING,      HiveGeometry.upCell(Alliance.BLUE, Tilt.FAR_UP));
        assertEquals(Cell.BLUE_AUDIENCE, HiveGeometry.upCell(Alliance.BLUE, Tilt.AUDIENCE_UP));
    }

    @Test public void tippingSwapsTheUpCell() {
        assertEquals(Tilt.AUDIENCE_UP, Tilt.FAR_UP.afterTip());
        assertEquals(Tilt.FAR_UP, Tilt.AUDIENCE_UP.afterTip());
    }

    @Test public void symmetryWithMadeUpNumbers() {
        // Pretend the red audience opening is at (60, 50), 40 in. up.
        assertArrayEquals(new double[] {60, 50, 40}, HiveGeometry.openingOf(Cell.RED_AUDIENCE, 60, 50, 40), EPS);
        assertArrayEquals(new double[] {60, 94, 40}, HiveGeometry.openingOf(Cell.RED_SCORING, 60, 50, 40), EPS);
        assertArrayEquals(new double[] {84, 94, 40}, HiveGeometry.openingOf(Cell.BLUE_SCORING, 60, 50, 40), EPS);
        assertArrayEquals(new double[] {84, 50, 40}, HiveGeometry.openingOf(Cell.BLUE_AUDIENCE, 60, 50, 40), EPS);
    }

    // TODO(measure): SKIPPED until RED_AUDIENCE_OPENING_X/Y/Z are filled in,
    // then it sanity-checks the numbers you typed.
    @Test public void openingLooksRight() {
        assumeTrue("Measure RED_AUDIENCE_OPENING_X/Y/Z in HiveGeometry from the field CAD",
                HiveGeometry.isMeasured());
        double[] p = HiveGeometry.aimPoint(Alliance.RED, Tilt.AUDIENCE_UP);
        assertTrue("The audience cell should be on the audience half (y < 72)", p[1] < 72);
        assertTrue("The opening should be inside the hive's footprint", FieldConstants.HIVE.contains(p[0], p[1]));
    }

    // ---- sideCell: target the cell on our side of the field (default SIDE_DEADBAND 6) ----

    @Test public void sideCell_audienceHalfPicksAudienceCell() {
        assertEquals(Cell.RED_AUDIENCE, HiveGeometry.sideCell(Alliance.RED, 30, null));
        assertEquals(Cell.BLUE_AUDIENCE, HiveGeometry.sideCell(Alliance.BLUE, 30, null));
    }

    @Test public void sideCell_farHalfPicksScoringCell() {
        assertEquals(Cell.RED_SCORING, HiveGeometry.sideCell(Alliance.RED, 110, null));
        assertEquals(Cell.BLUE_SCORING, HiveGeometry.sideCell(Alliance.BLUE, 110, null));
    }

    @Test public void sideCell_keepsCurrentInsideTheDeadband() {
        assertEquals(Cell.RED_AUDIENCE, HiveGeometry.sideCell(Alliance.RED, 76, Cell.RED_AUDIENCE));
        assertEquals(Cell.RED_SCORING, HiveGeometry.sideCell(Alliance.RED, 68, Cell.RED_SCORING));
        // Past the band, it switches.
        assertEquals(Cell.RED_SCORING, HiveGeometry.sideCell(Alliance.RED, 79, Cell.RED_AUDIENCE));
    }

    @Test public void sideCell_ignoresTheOtherAlliancesCell() {
        // Currently "targeting" a blue cell as red makes no sense: decide fresh.
        assertEquals(Cell.RED_AUDIENCE, HiveGeometry.sideCell(Alliance.RED, 70, Cell.BLUE_SCORING));
    }
}
