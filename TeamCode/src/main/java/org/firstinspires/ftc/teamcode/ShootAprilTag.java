package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShootAprilTag {
    private RobotHardware hardware;
    private double SHOOTER_VELOCITY_RPM = 2000; // RPM, tune with testing
    private static final double TICKS_PER_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    // Resolved 2026-10-29: 3:1, the HD Hex's internal gearbox. Sourced from
    // RobotHardware so there is exactly one place to fix it.
    private static final double SHOOTER_GEAR_RATIO = RobotHardware.SHOOTER_GEAR_RATIO;

    // Firing sequence timings. Every one of these is a starting guess until
    // somebody watches the real shooter -- see the TODO on shoot().
    private static final int BALLS_PER_VOLLEY = 3;
    private static final double SPINUP_TIME = 1.0;        // sec to reach speed before feeding
    private static final double INDEXER_FEED_TIME = 0.25; // sec of indexer run per ball
    private static final double SHOT_RECOVERY_TIME = 0.4; // sec for flywheels to recover
    private static final double INDEXER_FEED_POWER = 1.0;

    public ShootAprilTag(RobotHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * Align, spin up, and fire a volley.
     *
     * TODO(mechanism): this is a direct translation of the old flipper-based
     * sequence onto the new hardware -- "flipper flicks a ball in" became
     * "indexer motor feeds a ball in". Nobody has confirmed that is how the
     * real shooter loads, so treat the whole sequence as a first draft:
     *   - Does the indexer need to run continuously instead of pulsing per ball?
     *   - Should the collector keep feeding the indexer during a volley?
     *   - Does the pivot need to be at pollen/nectar spacing before firing?
     * Fix the sequence before tuning any of the timing constants.
     */
    public void shoot(int tagId) {
        // Step 1: Align to AprilTag (ID 20 or 24, center in view)
        alignToTag(tagId);

        // Step 2: Spin both flywheels up to the distance-derived speed
        double distance = getDistanceToTag(tagId);
        SHOOTER_VELOCITY_RPM = calculateVelocity(distance);
        double shooterTicksPerSec = SHOOTER_VELOCITY_RPM * TICKS_PER_REV * SHOOTER_GEAR_RATIO / 60.0;
        hardware.shooterLeft.setVelocity(shooterTicksPerSec);
        hardware.shooterRight.setVelocity(shooterTicksPerSec);

        // Step 3: Let them actually get there. Feeding a ball into a
        // still-accelerating flywheel is the classic cause of one good shot
        // followed by two short ones.
        waitSec(SPINUP_TIME);

        // Step 4: Pulse the indexer once per ball, pausing in between so the
        // flywheels recover the speed each ball steals on its way out.
        for (int i = 0; i < BALLS_PER_VOLLEY; i++) {
            hardware.indexer.setPower(INDEXER_FEED_POWER);
            waitSec(INDEXER_FEED_TIME);
            hardware.indexer.setPower(0);
            waitSec(SHOT_RECOVERY_TIME);
        }

        hardware.shooterLeft.setPower(0);
        hardware.shooterRight.setPower(0);
        hardware.indexer.setPower(0);
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