package org.firstinspires.ftc.teamcode.geometry;

/*
 * Field-centric drive math: turn "the stick, as the DRIVER sees it" into
 * "forward / strafe, as the ROBOT sees it".
 *
 * The driver stands behind their alliance wall. Stick forward means "away from
 * me", whichever way the robot is facing. awayHeading is the field heading of
 * "away from me" (Alliance.awayFromDriversHeading()), and robotHeading is
 * where the robot is facing now (Pedro's heading). Only the DIFFERENCE between
 * them matters: the stick gets rotated by that much.
 *
 * Plain numbers, no hardware, so it's tested on the laptop (FieldCentricTest).
 */
public final class FieldCentric {

    private FieldCentric() {}

    /**
     * @param stickForward  push up = +1 (that's -gamepad.left_stick_y)
     * @param stickRight    push right = +1 (gamepad.left_stick_x)
     * @param robotHeading  radians, CCW positive (Pedro)
     * @param awayHeading   radians, the field heading straight away from the driver
     * @return {robotForward, robotRight}
     */
    public static double[] toRobot(double stickForward, double stickRight,
                                   double robotHeading, double awayHeading) {
        // How far the robot is turned LEFT of "away from the driver".
        double d = robotHeading - awayHeading;
        double cos = Math.cos(d);
        double sin = Math.sin(d);
        double robotForward = stickForward * cos - stickRight * sin;
        double robotRight = stickForward * sin + stickRight * cos;
        return new double[] {robotForward, robotRight};
    }
}
