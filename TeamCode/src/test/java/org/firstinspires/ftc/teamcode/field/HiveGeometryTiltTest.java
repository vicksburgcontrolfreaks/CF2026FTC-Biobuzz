package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * tiltWithRaised is the inverse of upCell, and the wall re-anchor depends on it:
 * it is how DriveTeleOp gets the hive tilt without detection.robotPose, which is
 * null on the match camera. Getting it backwards would place every tag on the
 * wrong side of the pivot (~1.4 in sideways, and the wrong cell entirely).
 */
public class HiveGeometryTiltTest {

    @Test
    public void itIsExactlyTheInverseOfUpCell() {
        for (Alliance a : Alliance.values()) {
            for (Tilt t : Tilt.values()) {
                assertEquals("round trip " + a + " " + t,
                        t, HiveGeometry.tiltWithRaised(HiveGeometry.upCell(a, t)));
            }
        }
    }

    @Test
    public void scoringCellsAreUpWhenTheFarSideIsUp() {
        assertEquals(Tilt.FAR_UP, HiveGeometry.tiltWithRaised(Cell.RED_SCORING));
        assertEquals(Tilt.FAR_UP, HiveGeometry.tiltWithRaised(Cell.BLUE_SCORING));
    }

    @Test
    public void audienceCellsAreUpWhenTheAudienceSideIsUp() {
        assertEquals(Tilt.AUDIENCE_UP, HiveGeometry.tiltWithRaised(Cell.RED_AUDIENCE));
        assertEquals(Tilt.AUDIENCE_UP, HiveGeometry.tiltWithRaised(Cell.BLUE_AUDIENCE));
    }

    @Test
    public void theAnswerDoesNotDependOnAlliance() {
        // Both hives tip about the same field-x axis, so the mapping is shared.
        assertEquals(HiveGeometry.tiltWithRaised(Cell.RED_SCORING),
                HiveGeometry.tiltWithRaised(Cell.BLUE_SCORING));
        assertEquals(HiveGeometry.tiltWithRaised(Cell.RED_AUDIENCE),
                HiveGeometry.tiltWithRaised(Cell.BLUE_AUDIENCE));
    }
}
