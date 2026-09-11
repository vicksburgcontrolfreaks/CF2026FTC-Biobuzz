package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShootAprilTag {
    private RobotHardware hardware;
    private double SHOOTER_VELOCITY_RPM = 2000; // RPM, tune with testing
    private double COLLECTOR_VELOCITY_RPM = 1000; // RPM for flywheel mode
    private double FLIP_POSITION_LAUNCH = 0.5; // Servo pos for launch
    private double FLIP_POSITION_RESET = 0.0; // Reset pos
    private double REVERSE_TIME = 0.5; // Sec to reverse collector
    private static final double TICKS_PER_REV = 28; // REV HD Hex Motor shaft
    private static final double COLLECTOR_GEAR_RATIO = 20.0; // Collector 20:1
    private static final double SHOOTER_GEAR_RATIO = 1.0; // Shooter no gearbox

    public ShootAprilTag(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void shoot(int tagId) {
        // Step 1: Align to AprilTag (ID 20 or 24, center in view)
        alignToTag(tagId);

        // Step 2: Shooting sequence for 3 balls
        double distance = getDistanceToTag(tagId);
        SHOOTER_VELOCITY_RPM = calculateVelocity(distance);
        double shooterTicksPerSec = SHOOTER_VELOCITY_RPM * TICKS_PER_REV * SHOOTER_GEAR_RATIO / 60.0;
        hardware.shooter.setVelocity(shooterTicksPerSec);

        for (int i = 0; i < 3; i++) {
            double collectorTicksPerSec = COLLECTOR_VELOCITY_RPM * TICKS_PER_REV * COLLECTOR_GEAR_RATIO / 60.0;
            hardware.collector.setVelocity(collectorTicksPerSec);
            hardware.flipper.setPosition(FLIP_POSITION_LAUNCH);
            waitSec(0.5);
            hardware.collector.setPower(-0.3);
            waitSec(REVERSE_TIME);
            hardware.collector.setPower(0);
            hardware.flipper.setPosition(FLIP_POSITION_RESET);
            waitSec(0.2);
        }

        hardware.shooter.setPower(0);
        hardware.collector.setPower(0);
    }

    public void alignRotationOnly(int tagId) {
        // PID to center tag by rotation only (x=0, y=0)
        double kp = 0.01, ki = 0.0, kd = 0.0;
        double prevErrorX = 0, integralX = 0;
        double prevErrorY = 0, integralY = 0;
        Timer timeout = new Timer();
        double TIMEOUT_SEC = 5.0;

        while (timeout.getElapsedTimeSeconds() < TIMEOUT_SEC) {
            AprilTagDetection detection = getDetection(tagId);
            if (detection == null) {
                hardware.visionPortal.getCameraState();
                continue;
            }

            // Errors: Aim for x=0, y=0 (center on screen)
            double errorX = detection.ftcPose.x;
            double errorY = detection.ftcPose.y;

            integralX += errorX;
            integralY += errorY;

            double derivX = errorX - prevErrorX;
            double derivY = errorY - prevErrorY;

            // Map x/y errors to rotation (turn)
            double turn = kp * (errorX + errorY) + ki * (integralX + integralY) + kd * (derivX + derivY);
            double strafe = 0.0; // No lateral movement
            double forward = 0.0; // No forward/back movement

            // Drive powers (rotation only)
            hardware.lf.setPower(turn);
            hardware.rf.setPower(-turn);
            hardware.lr.setPower(turn);
            hardware.rr.setPower(-turn);

            prevErrorX = errorX;
            prevErrorY = errorY;

            if (Math.abs(errorX) < 1 && Math.abs(errorY) < 1) break; // Threshold
        }

        // Stop drive
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    private void alignToTag(int tagId) {
        // Original full alignment (x, y, yaw)
        double kp = 0.1, ki = 0.0, kd = 0.0;
        double prevErrorX = 0, integralX = 0;
        double prevErrorY = 0, integralY = 0;
        Timer timeout = new Timer();
        double TIMEOUT_SEC = 2.5;

        while (timeout.getElapsedTimeSeconds() < TIMEOUT_SEC) {
            AprilTagDetection detection = getDetection(tagId);
            if (detection == null) {
                hardware.visionPortal.getCameraState();
                continue;
            }

            double errorX = detection.ftcPose.x;
            double errorY = detection.ftcPose.y;

            integralX += errorX;
            integralY += errorY;

            double derivX = errorX - prevErrorX;
            double derivY = errorY - prevErrorY;

            double strafe = kp * errorX + ki * integralX + kd * derivX;
            double forward = kp * errorY + ki * integralY + kd * derivY;
            double turn = 0.0;

            hardware.lf.setPower(forward + strafe + turn);
            hardware.rf.setPower(forward - strafe - turn);
            hardware.lr.setPower(forward - strafe + turn);
            hardware.rr.setPower(forward + strafe - turn);

            prevErrorX = errorX;
            prevErrorY = errorY;

            if (Math.abs(errorX) < 1 && Math.abs(errorY) < 1) break;
        }

        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    private AprilTagDetection getDetection(int tagId) {
        for (AprilTagDetection detection : hardware.aprilTagProcessor.getDetections()) {
            if (detection.id == tagId) return detection;
        }
        return null;
    }

    private double getDistanceToTag(int tagId) {
        AprilTagDetection detection = getDetection(tagId);
        return detection != null ? detection.ftcPose.range : 0;
    }

    private double calculateVelocity(double distance) {
        return 1500 + 50 * distance; // RPM, TBD
    }

    private void waitSec(double sec) {
        long ms = (long) (sec * 1000);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // Ignore
        }
    }
}