package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.geometry.Rigid3;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/*
 * These check the MODEL in HiveTags, not its made-up numbers: they should
 * still pass after the real measurements go in.
 */
public class HiveTagsTest {

    private static final double EPS = 1e-6;

    @Test public void tagsFaceDownwardInBothTilts() {
        // Tag z points INTO the tag; the tags face down, so z points up-ish,
        // tipped by at most half the swing.
        double minUp = Math.cos(Math.toRadians(HiveTags.HIVE_SWING_DEG / 2)) - EPS;
        for (Cell cell : Cell.values()) {
            for (int id : cell.tagIds) {
                for (Tilt tilt : Tilt.values()) {
                    assertTrue("tag " + id + " " + tilt, HiveTags.tagPose(id, tilt).axis(2)[2] >= minUp);
                }
            }
        }
    }

    @Test public void farUpLiftsTheFarCell() {
        double far = HiveTags.tagPose(Cell.RED_SCORING.tagIds[0], Tilt.FAR_UP).z();
        double audience = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.FAR_UP).z();
        assertTrue(far > audience);
        far = HiveTags.tagPose(Cell.RED_SCORING.tagIds[0], Tilt.AUDIENCE_UP).z();
        audience = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.AUDIENCE_UP).z();
        assertTrue(audience > far);
    }

    @Test public void tiltChangeMovesATagToItsOtherPose() {
        for (int id : Cell.BLUE_AUDIENCE.tagIds) {
            Rigid3 moved = HiveTags.tiltChange(Tilt.FAR_UP, Tilt.AUDIENCE_UP).times(HiveTags.tagPose(id, Tilt.FAR_UP));
            Rigid3 direct = HiveTags.tagPose(id, Tilt.AUDIENCE_UP);
            assertEquals(direct.x(), moved.x(), EPS);
            assertEquals(direct.y(), moved.y(), EPS);
            assertEquals(direct.z(), moved.z(), EPS);
            assertEquals(1.0, dot(direct.axis(2), moved.axis(2)), EPS);
        }
    }

    @Test public void blueIsRedSpun180() {
        // Spinning the field 180 deg sends red audience -> blue scoring, and a tip
        // that lifts red's audience cell lifts blue's far cell.
        for (int k = 0; k < 4; k++) {
            Rigid3 red = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[k], Tilt.AUDIENCE_UP);
            Rigid3 blue = HiveTags.tagPose(Cell.BLUE_SCORING.tagIds[k], Tilt.FAR_UP);
            assertEquals(FieldConstants.FIELD_SIZE - red.x(), blue.x(), EPS);
            assertEquals(FieldConstants.FIELD_SIZE - red.y(), blue.y(), EPS);
            assertEquals(red.z(), blue.z(), EPS);
        }
    }

    @Test public void stickerBottomEdgePointsTowardThePivot() {
        // Tag "down" (y) runs toward the hive's middle, per manual Figure 9-16.
        assertTrue(HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.FAR_UP).axis(1)[1] > 0);
        assertTrue(HiveTags.tagPose(Cell.RED_SCORING.tagIds[0], Tilt.FAR_UP).axis(1)[1] < 0);
    }

    @Test public void stickerRowMatchesManualFigure915() {
        // Read from the manual by Evan 10/27: 6.5 in. from centerline to each end
        // tag, 3.75 in. between the two tags on the same side.
        assertEquals(-6.5,  HiveTags.rowOffset(0), EPS);
        assertEquals(-2.75, HiveTags.rowOffset(1), EPS);
        assertEquals(+2.75, HiveTags.rowOffset(2), EPS);
        assertEquals(+6.5,  HiveTags.rowOffset(3), EPS);
        assertEquals(13.0, HiveTags.rowOffset(3) - HiveTags.rowOffset(0), EPS);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    /*
     * Unlike the rest of this file, this one DOES pin a measurement: Shane
     * levelled off the pivot axis on 2026-11-11 and read the raised audience
     * cell's tag centre at +5.75 in., bottom edge +5.0, top edge +6.5.
     *
     * CELL_DROP was solved from that number, so it is the thing keeping the tag
     * model honest. If a later CAD figure moves CELL_ARM or HIVE_SWING_DEG,
     * CELL_DROP has to move with it or this fails -- which is the point.
     */
    @Test public void audienceTagSitsWhereShaneMeasuredIt() {
        double aboveSticker = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.AUDIENCE_UP).z()
                - FieldConstants.HIVE_PIVOT_HEIGHT;
        assertEquals("tag centre above the pivot level", 5.75, aboveSticker, 0.05);

        // Every tag on a cell is on one sticker in a row along field x, so they
        // all sit at the same height. A row built along the wrong axis would
        // show up here as tags at different heights.
        for (int id : Cell.RED_AUDIENCE.tagIds) {
            assertEquals("tag " + id + " height", HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.AUDIENCE_UP).z(),
                    HiveTags.tagPose(id, Tilt.AUDIENCE_UP).z(), EPS);
        }
    }

    /*
     * The tag's own vertical span dates the tilt independently of anyone's
     * protractor: a 3.25 in. tag whose edges read +5.0 and +6.5 rises 1.5 in.,
     * so asin(1.5 / 3.25) = 27.5 deg. That is within tape error of the 30 deg
     * HIVE_SWING_DEG is built on, so this allows a couple of degrees either way.
     */
    @Test public void tagRiseAgreesWithTheSwingAngle() {
        double halfSwing = HiveTags.HIVE_SWING_DEG / 2;
        double riseFromMeasurement = Math.toDegrees(Math.asin(1.5 / FieldAprilTags.TAG_SIZE));
        assertEquals("tilt from the tag's own span vs HIVE_SWING_DEG/2",
                halfSwing, riseFromMeasurement, 3.0);
    }
}
