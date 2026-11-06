package org.firstinspires.ftc.teamcode.geometry;

import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

/*
 * Rigid3 has to agree with the SDK, or every pose TagLocalizer makes is wrong
 * in a way that only shows up on the field. These tests pin each convention to
 * something the SDK itself says or uses.
 */
public class Rigid3Test {

    private static final double EPS = 1e-9;

    private static void assertSame(Rigid3 a, Rigid3 b) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals("R" + i + j, a.get(i, j), b.get(i, j), 1e-6);
            }
        }
        assertEquals(a.x(), b.x(), 1e-6);
        assertEquals(a.y(), b.y(), 1e-6);
        assertEquals(a.z(), b.z(), 1e-6);
    }

    @Test public void inverseUndoes() {
        Rigid3 a = Rigid3.fromPose3D(10, -4, 7, 33, -12, 71);
        assertSame(Rigid3.identity(), a.times(a.inverse()));
        assertSame(Rigid3.identity(), a.inverse().times(a));
    }

    @Test public void yawPitchRollRoundTrip() {
        double[][] cases = {{0, 0, 0}, {90, 0, 0}, {33, -12, 71}, {-150, 40, -100}};
        for (double[] ypr : cases) {
            double[] back = Rigid3.fromPose3D(0, 0, 0, ypr[0], ypr[1], ypr[2]).toYawPitchRoll();
            assertArrayEquals(ypr, back, 1e-9);
        }
    }

    @Test public void yawIsCounterClockwiseAboutUp() {
        // Yaw 90: the frame's x axis now points along +y.
        assertArrayEquals(new double[] {0, 1, 0}, Rigid3.fromPose3D(0, 0, 0, 90, 0, 0).axis(0), EPS);
    }

    @Test public void cameraPitchMinus90LooksStraightAhead() {
        // FTC docs: setCameraPose with pitch -90 = a level camera looking along
        // the robot's +Y (forward). Camera frame is OpenCV: z out the lens, y down.
        Rigid3 cam = Rigid3.fromCameraPose(0, 0, 0, 0, -90, 0);
        assertArrayEquals(new double[] {0, 1, 0}, cam.axis(2), EPS);    // lens looks forward
        assertArrayEquals(new double[] {0, 0, -1}, cam.axis(1), EPS);   // image "down" is down
        assertArrayEquals(new double[] {1, 0, 0}, cam.axis(0), EPS);    // image "right" is right
    }

    @Test public void tagFrameConventionMatchesTheSdkDatabase() {
        // DECODE tag 12 sits on the wall at y = +70.25, facing into the field.
        // Its SDK quaternion is (0.7071, -0.7071, 0, 0). If tag z is INTO the tag
        // and y is the tag's "down", that rotation must send z to +y (into the
        // wall) and y to -z (down). HiveTags is built on exactly this.
        double h = Math.sqrt(0.5);
        Rigid3 tag12 = Rigid3.fromQuaternion(h, -h, 0, 0);
        assertArrayEquals(new double[] {0, 1, 0}, tag12.axis(2), EPS);
        assertArrayEquals(new double[] {0, 0, -1}, tag12.axis(1), EPS);
        assertArrayEquals(new double[] {1, 0, 0}, tag12.axis(0), EPS);
    }

    @Test public void quaternionRoundTrip() {
        Rigid3 a = Rigid3.fromPose3D(0, 0, 0, 33, -12, 71);
        double[] q = a.toQuaternion();
        assertSame(a, Rigid3.fromQuaternion(q[0], q[1], q[2], q[3]));
        Rigid3 b = Rigid3.fromPose3D(0, 0, 0, 180, 0, 0);   // trace < 0 branch
        q = b.toQuaternion();
        assertSame(b, Rigid3.fromQuaternion(q[0], q[1], q[2], q[3]));
    }

    @Test public void rotationAboutALineKeepsTheLineFixed() {
        Rigid3 tip = Rigid3.rotXAbout(72, 44, Math.toRadians(20));
        assertArrayEquals(new double[] {5, 72, 44}, tip.apply(5, 72, 44), EPS);
        // A point 10 in. toward +y from the line swings UP.
        double[] p = tip.apply(0, 82, 44);
        assertEquals(10 * Math.sin(Math.toRadians(20)), p[2] - 44, EPS);
    }
}
