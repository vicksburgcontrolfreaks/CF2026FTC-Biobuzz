package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.FieldGuard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * AprilTag Field Follow: exactly like AprilTag Follow, but the robot also
 * keeps track of WHERE IT IS on the field, and won't drive itself into the
 * hive, a flower, or a wall. If the tag leads it toward one, it slides along
 * the edge instead (see field/FieldGuard).
 *
 * SETUP: put the robot where Red Start Up starts -- (59, 8), facing 180 --
 * then press INIT, then Start. Its position is reset to that spot at Start.
 *
 * It does NOT drive AROUND obstacles. If the hive is between you and the
 * robot, it stops at the edge and waits.
 */
@TeleOp(name = "AprilTag Field Follow", group = "Test")
public class AprilTagFieldFollow extends AprilTagFollow {

    // Same start as Red Start Up
    private static final Pose START_POSE = new Pose(59, 8, Math.toRadians(180));

    // Slower than the plain version, so it can stop before an obstacle
    private static final double FIELD_SPEED_CAP = 0.35;

    // Pedro's odometry (the Pinpoint), used ONLY to know where we are.
    // Same settings as the autos. It never drives the motors.
    private PinpointLocalizer odometry;

    @Override
    protected void onInit() {
        odometry = new PinpointLocalizer(hardwareMap, Constants.localizerConstants, START_POSE);
        telemetry.addLine("FIELD MODE: robot must start at (59, 8) facing 180");
    }

    @Override
    protected void onStart() {
        // In case the robot got nudged between INIT and Start
        odometry.setPose(START_POSE);
    }

    @Override
    protected double speedCap() {
        return FIELD_SPEED_CAP;
    }

    @Override
    protected double[] limitMove(double drive, double strafe) {
        odometry.update(); // every loop, or the position goes stale
        Pose pose = odometry.getPose();

        if (odometry.isNAN()) {
            // Odometry is broken -> the guard can't work -> don't move
            telemetry.addLine("ODOMETRY ERROR -> stopped (check the Pinpoint)");
            return new double[]{0, 0};
        }

        // Turn the robot's move (forward/left) into a move on the FIELD (x/y).
        // Facing 'heading', forward is (cos, sin) and left is (-sin, cos).
        double heading = pose.getHeading();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double moveX = drive * cos - strafe * sin;
        double moveY = drive * sin + strafe * cos;

        Pose velocity = odometry.getVelocity();
        double speed = Math.hypot(velocity.getX(), velocity.getY()); // in/s

        FieldGuard.Result allowed = FieldGuard.check(pose.getX(), pose.getY(), moveX, moveY, speed);

        // And back from field x/y to the robot's forward/left
        double newDrive  =  allowed.moveX * cos + allowed.moveY * sin;
        double newStrafe = -allowed.moveX * sin + allowed.moveY * cos;

        telemetry.addData("Robot at", "(%.1f, %.1f) facing %.0f deg",
                pose.getX(), pose.getY(), Math.toDegrees(heading));
        telemetry.addData("Guard", allowed.blockedBy == null
                ? "clear" : "BLOCKED by " + allowed.blockedBy);

        return new double[]{newDrive, newStrafe};
    }
}
