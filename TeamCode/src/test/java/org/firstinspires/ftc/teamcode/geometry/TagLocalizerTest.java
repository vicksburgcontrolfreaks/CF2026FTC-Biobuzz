package org.firstinspires.ftc.teamcode.geometry;

import org.firstinspires.ftc.teamcode.field.FieldAprilTags.Cell;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.field.HiveTags;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * TagLocalizer's tilt fix, checked against a fake camera.
 *
 * Each test builds a made-up world: a turret at a known spot, a hive tipped a
 * known way. It works out what the camera would really see (tagInCamera), then
 * runs the SDK's computeRobotPose on it -- copied line for line from
 * AprilTagProcessorImpl (SDK 11.2.1), using the REFERENCE-tilt tag library just
 * like the robot will. TagLocalizer.solveTurret has to recover the turret we
 * started with AND the tilt the hive was really in.
 */
public class TagLocalizerTest {

    private static final double POS = 1e-4;   // inches
    private static final double ANG = 1e-4;   // degrees

    /** The SDK frame for a turret base on the floor at (x, y), pointing toward field heading h. */
    private static Rigid3 turretAt(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double[] forward = {Math.cos(h), Math.sin(h), 0};   // SDK +Y
        double[] right = {Math.sin(h), -Math.cos(h), 0};    // SDK +X
        return Rigid3.fromAxes(x, y, 0, right, forward, new double[] {0, 0, 1});
    }

    /** The turret base pointed at a cell's middle from (x, y). */
    private static Rigid3 turretAimedAt(double x, double y, Cell cell, Tilt tilt) {
        Rigid3 tag = HiveTags.tagPose(cell.tagIds[0], tilt);
        return turretAt(x, y, Math.toDegrees(Math.atan2(tag.y() - y, tag.x() - x)));
    }

    /** What the camera measures: the tag, as seen from the camera. */
    private static Rigid3 tagInCamera(Rigid3 trueTurret, Rigid3 cameraOnTurret, int tagId, Tilt trueTilt) {
        Rigid3 cameraInField = trueTurret.times(cameraOnTurret);
        return cameraInField.inverse().times(HiveTags.tagPose(tagId, trueTilt));
    }

    /** AprilTagProcessorImpl.computeRobotPose, then the yaw/pitch/roll trip through Pose3D. */
    private static Rigid3 sdkRobotPose(Rigid3 tagInCamera, int tagId) {
        Rigid3 tagInFieldFrame = HiveTags.tagPose(tagId, HiveTags.REFERENCE);   // the library's pose
        Rigid3 cameraInTagFrame = tagInCamera.inverse();
        Rigid3 robotInCameraFrame = TagLocalizer.cameraOnTurret().inverse();
        Rigid3 robotInFieldFrame = tagInFieldFrame.times(cameraInTagFrame).times(robotInCameraFrame);
        double[] ypr = robotInFieldFrame.toYawPitchRoll();
        return Rigid3.fromPose3D(robotInFieldFrame.x(), robotInFieldFrame.y(), robotInFieldFrame.z(),
                ypr[0], ypr[1], ypr[2]);
    }

    /** Sanity: the made-up scene must be one a real camera could see. */
    private static void assertVisible(Rigid3 tagInCamera) {
        assertTrue("tag is behind the camera", tagInCamera.z() > 0);
        assertTrue("camera sees the back of the tag", tagInCamera.axis(2)[2] > 0);
        double offAxis = Math.toDegrees(Math.atan2(Math.hypot(tagInCamera.x(), tagInCamera.y()), tagInCamera.z()));
        assertTrue("tag is outside a ~70 deg field of view (" + offAxis + ")", offAxis < 35);
    }

    private static void assertRecovers(double x, double y, Cell cell, Tilt trueTilt) {
        Rigid3 truth = turretAimedAt(x, y, cell, trueTilt);
        for (int id : cell.tagIds) {
            Rigid3 seen = tagInCamera(truth, TagLocalizer.cameraOnTurret(), id, trueTilt);
            assertVisible(seen);
            TagLocalizer.TurretFix fix = TagLocalizer.solveTurret(sdkRobotPose(seen, id));

            String where = cell + " tag " + id + " from (" + x + "," + y + "), hive " + trueTilt;
            assertEquals(where, trueTilt, fix.hiveTilt);
            assertTrue(where, fix.believable);
            assertEquals(where, truth.x(), fix.x, POS);
            assertEquals(where, truth.y(), fix.y, POS);
            double[] forward = truth.axis(1);
            double trueHeading = Math.toDegrees(Math.atan2(forward[1], forward[0]));
            assertEquals(where, 0, TurretGeometryTestHelp.wrap(Math.toDegrees(fix.headingRad) - trueHeading), ANG);
            assertEquals(where, 0, fix.levelErrorDeg, 1e-3);
        }
    }

    @Test public void hiveInTheReferenceTilt() {
        assertRecovers(60, 24, Cell.RED_AUDIENCE, HiveTags.REFERENCE);
    }

    @Test public void hiveTippedTheOtherWay() {
        // The SDK's answer is wrong here (its library assumes the other tilt);
        // the fix has to rotate it back.
        assertRecovers(60, 24, Cell.RED_AUDIENCE, Tilt.AUDIENCE_UP);
    }

    @Test public void fromAnAngleAndOtherCells() {
        assertRecovers(30, 36, Cell.RED_AUDIENCE, Tilt.AUDIENCE_UP);
        assertRecovers(60, 120, Cell.RED_FAR, Tilt.FAR_UP);
        assertRecovers(60, 120, Cell.RED_FAR, Tilt.AUDIENCE_UP);
        assertRecovers(100, 24, Cell.BLUE_AUDIENCE, Tilt.FAR_UP);
        assertRecovers(110, 115, Cell.BLUE_FAR, Tilt.AUDIENCE_UP);
    }

    @Test public void wrongTiltIsWayOffLevel() {
        // The losing candidate should miss by the whole swing, so the margin
        // over LEVEL_TOLERANCE_DEG is large and the choice is never a coin flip.
        Rigid3 truth = turretAimedAt(60, 24, Cell.RED_AUDIENCE, Tilt.AUDIENCE_UP);
        int id = Cell.RED_AUDIENCE.tagIds[0];
        Rigid3 sdk = sdkRobotPose(tagInCamera(truth, TagLocalizer.cameraOnTurret(), id, Tilt.AUDIENCE_UP), id);
        Rigid3 wrong = HiveTags.tiltChange(HiveTags.REFERENCE, Tilt.FAR_UP).times(sdk);
        double wrongLevelError = Math.toDegrees(Math.acos(wrong.get(2, 2)));
        assertEquals(HiveTags.HIVE_SWING_DEG, wrongLevelError, 1e-6);
    }

    @Test public void badCameraMeasurementIsNotBelieved() {
        // The real camera is tipped up 45, but the code thinks 30. The answer
        // comes out tipped 15 deg -- it must be rejected, not quietly trusted.
        double saved = TagLocalizer.CAMERA_PITCH_DEG;
        Rigid3 realCamera;
        try {
            TagLocalizer.CAMERA_PITCH_DEG = 45;
            realCamera = TagLocalizer.cameraOnTurret();
        } finally {
            TagLocalizer.CAMERA_PITCH_DEG = saved;
        }
        Rigid3 truth = turretAimedAt(60, 30, Cell.RED_AUDIENCE, Tilt.FAR_UP);
        int id = Cell.RED_AUDIENCE.tagIds[0];
        TagLocalizer.TurretFix fix = TagLocalizer.solveTurret(sdkRobotPose(tagInCamera(truth, realCamera, id, Tilt.FAR_UP), id));
        assertFalse("a 15 deg camera error should fail the level check", fix.believable);
    }
}

/** wrapDegrees without depending on the students' TurretGeometry step 1. */
final class TurretGeometryTestHelp {
    static double wrap(double deg) {
        double d = deg % 360;
        if (d <= -180) d += 360;
        if (d > 180) d -= 360;
        return d;
    }
}
