package org.firstinspires.ftc.teamcode.geometry;

/*
 * Rigid3: a 3D position + orientation ("where is it and which way is it
 * turned"), as a 3x3 rotation R and a translation t. Applying it to a point p
 * gives R*p + t. Read a.times(b) as "b, described inside a's frame."
 *
 * MENTOR-LEVEL helper for TagLocalizer. It exists because the SDK's own
 * OpenGLMatrix calls android.opengl.Matrix, which can't run in a laptop test.
 *
 * The SDK conventions this file matches (read from SDK 11.2.1 source):
 *   - Pose3D orientations (robotPose): intrinsic ZXY,  R = Rz(yaw) Rx(pitch) Ry(roll)
 *   - setCameraPose orientation:       intrinsic ZXZ,  R = Rz(yaw) Rx(pitch) Rz(roll)
 *   - Tag frames: x = right, y = down, z = INTO the tag (away from the camera)
 *   - Quaternions: (w, x, y, z), the same formula as Quaternion.fromMatrix
 */
public final class Rigid3 {

    // Row-major rotation, then translation. Immutable.
    private final double[] r;
    private final double tx, ty, tz;

    private Rigid3(double[] r, double tx, double ty, double tz) {
        this.r = r;
        this.tx = tx;
        this.ty = ty;
        this.tz = tz;
    }

    public static Rigid3 identity() {
        return new Rigid3(new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, 0, 0, 0);
    }

    public static Rigid3 translation(double x, double y, double z) {
        return new Rigid3(new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, x, y, z);
    }

    /** A pure rotation from a row-major 3x3 matrix. */
    public static Rigid3 rotation(double... rowMajor) {
        if (rowMajor.length != 9) throw new IllegalArgumentException("need 9 numbers");
        return new Rigid3(rowMajor.clone(), 0, 0, 0);
    }

    /** A frame at (x, y, z) whose own x, y, z axes point along the given field directions. */
    public static Rigid3 fromAxes(double x, double y, double z, double[] xAxis, double[] yAxis, double[] zAxis) {
        return new Rigid3(new double[] {
                xAxis[0], yAxis[0], zAxis[0],
                xAxis[1], yAxis[1], zAxis[1],
                xAxis[2], yAxis[2], zAxis[2]}, x, y, z);
    }

    public static Rigid3 rotX(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return rotation(1, 0, 0, 0, c, -s, 0, s, c);
    }

    public static Rigid3 rotY(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return rotation(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    public static Rigid3 rotZ(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return rotation(c, -s, 0, s, c, 0, 0, 0, 1);
    }

    /** Rotation by `rad` about the line through (any x, y0, z0) running along +x. */
    public static Rigid3 rotXAbout(double y0, double z0, double rad) {
        return translation(0, y0, z0).times(rotX(rad)).times(translation(0, -y0, -z0));
    }

    /** What the SDK's Pose3D means: position plus intrinsic ZXY yaw/pitch/roll, in degrees. */
    public static Rigid3 fromPose3D(double x, double y, double z, double yawDeg, double pitchDeg, double rollDeg) {
        return translation(x, y, z)
                .times(rotZ(Math.toRadians(yawDeg)))
                .times(rotX(Math.toRadians(pitchDeg)))
                .times(rotY(Math.toRadians(rollDeg)));
    }

    /** What setCameraPose(position, YawPitchRollAngles) means: intrinsic ZXZ, degrees. */
    public static Rigid3 fromCameraPose(double x, double y, double z, double yawDeg, double pitchDeg, double rollDeg) {
        return translation(x, y, z)
                .times(rotZ(Math.toRadians(yawDeg)))
                .times(rotX(Math.toRadians(pitchDeg)))
                .times(rotZ(Math.toRadians(rollDeg)));
    }

    /** this, then other inside this frame. */
    public Rigid3 times(Rigid3 o) {
        double[] m = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                m[3 * i + j] = r[3 * i] * o.r[j] + r[3 * i + 1] * o.r[3 + j] + r[3 * i + 2] * o.r[6 + j];
            }
        }
        double[] t = apply(o.tx, o.ty, o.tz);
        return new Rigid3(m, t[0], t[1], t[2]);
    }

    public Rigid3 inverse() {
        // R is a rotation, so its inverse is its transpose.
        double[] m = {r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]};
        return new Rigid3(m,
                -(m[0] * tx + m[1] * ty + m[2] * tz),
                -(m[3] * tx + m[4] * ty + m[5] * tz),
                -(m[6] * tx + m[7] * ty + m[8] * tz));
    }

    public double[] apply(double x, double y, double z) {
        return new double[] {
                r[0] * x + r[1] * y + r[2] * z + tx,
                r[3] * x + r[4] * y + r[5] * z + ty,
                r[6] * x + r[7] * y + r[8] * z + tz};
    }

    /** Where this frame's own axis (0 = x, 1 = y, 2 = z) points, in the outer frame. */
    public double[] axis(int which) {
        return new double[] {r[which], r[3 + which], r[6 + which]};
    }

    public double get(int row, int col) { return r[3 * row + col]; }
    public double x() { return tx; }
    public double y() { return ty; }
    public double z() { return tz; }

    /** Intrinsic ZXY angles in degrees {yaw, pitch, roll}, as the SDK reports them in Pose3D. */
    public double[] toYawPitchRoll() {
        // R = Rz(a) Rx(b) Ry(c):  r21 = sin b,  r01 = -sin a cos b,  r11 = cos a cos b,
        //                         r20 = -cos b sin c,  r22 = cos b cos c
        double pitch = Math.asin(Math.max(-1, Math.min(1, r[7])));
        double yaw = Math.atan2(-r[1], r[4]);
        double roll = Math.atan2(-r[6], r[8]);
        return new double[] {Math.toDegrees(yaw), Math.toDegrees(pitch), Math.toDegrees(roll)};
    }

    /** {w, x, y, z}, same formula as the SDK's Quaternion.fromMatrix. For AprilTagLibrary.addTag. */
    public double[] toQuaternion() {
        double m00 = r[0], m01 = r[1], m02 = r[2];
        double m10 = r[3], m11 = r[4], m12 = r[5];
        double m20 = r[6], m21 = r[7], m22 = r[8];
        double tr = m00 + m11 + m22;
        double w, x, y, z;
        if (tr > 0) {
            double s = Math.sqrt(tr + 1.0) * 2;
            w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            double s = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
            w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
            w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
        } else {
            double s = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
            w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
        }
        return new double[] {w, x, y, z};
    }

    /** The rotation for a quaternion {w, x, y, z}, same formula as the SDK's Quaternion.toMatrix. */
    public static Rigid3 fromQuaternion(double w, double x, double y, double z) {
        return rotation(
                1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
                2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y));
    }

    @Override public String toString() {
        double[] ypr = toYawPitchRoll();
        return String.format("(%.2f, %.2f, %.2f) yaw %.1f pitch %.1f roll %.1f", tx, ty, tz, ypr[0], ypr[1], ypr[2]);
    }
}
