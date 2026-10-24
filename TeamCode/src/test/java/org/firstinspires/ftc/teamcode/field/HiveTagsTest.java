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
        double far = HiveTags.tagPose(Cell.RED_FAR.tagIds[0], Tilt.FAR_UP).z();
        double audience = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.FAR_UP).z();
        assertTrue(far > audience);
        far = HiveTags.tagPose(Cell.RED_FAR.tagIds[0], Tilt.AUDIENCE_UP).z();
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
        // Spinning the field 180 deg sends red audience -> blue far, and a tip
        // that lifts red's audience cell lifts blue's far cell.
        for (int k = 0; k < 4; k++) {
            Rigid3 red = HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[k], Tilt.AUDIENCE_UP);
            Rigid3 blue = HiveTags.tagPose(Cell.BLUE_FAR.tagIds[k], Tilt.FAR_UP);
            assertEquals(FieldConstants.FIELD_SIZE - red.x(), blue.x(), EPS);
            assertEquals(FieldConstants.FIELD_SIZE - red.y(), blue.y(), EPS);
            assertEquals(red.z(), blue.z(), EPS);
        }
    }

    @Test public void stickerBottomEdgePointsTowardThePivot() {
        // Tag "down" (y) runs toward the hive's middle, per manual Figure 9-16.
        assertTrue(HiveTags.tagPose(Cell.RED_AUDIENCE.tagIds[0], Tilt.FAR_UP).axis(1)[1] > 0);
        assertTrue(HiveTags.tagPose(Cell.RED_FAR.tagIds[0], Tilt.FAR_UP).axis(1)[1] < 0);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
}
