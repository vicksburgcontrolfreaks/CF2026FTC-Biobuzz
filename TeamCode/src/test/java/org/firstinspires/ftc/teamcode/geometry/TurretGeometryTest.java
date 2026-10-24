package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * Tests for TurretGeometry. These run on a laptop, no robot needed:
 *
 *     gradlew :TeamCode:testDebugUnitTest
 *
 * (In Android Studio you can also right-click this file -> Run.)
 *
 * They are grouped by the STEP numbers in TurretGeometry. Make step 1's tests
 * pass before starting step 2 -- later steps use the earlier ones.
 *
 * DRAW EVERY TEST BEFORE YOU DEBUG IT. Graph paper, +x to the right, +y up,
 * the robot as an arrow. If your drawing disagrees with the expected number,
 * tell a mentor -- the test could be wrong too.
 *
 * Every test makes its own made-up robot, so changing the real measurements
 * in TurretGeometry never breaks these.
 */
public class TurretGeometryTest {

    private static final double EPS = 1e-6;   // "close enough" for exact math
    private static final double DEG = 1e-3;   // "close enough" for angles in degrees

    // The simplest possible robot: turret right on the center, 0 deg = straight
    // ahead, bigger angle = counter-clockwise. Steps 2-3 start here.
    private static final TurretGeometry SIMPLE = new TurretGeometry(0, 0, 0, +1);

    // A robot shaped more like ours: turret 6 in. behind center, and turret
    // 0 deg faces BACKWARD.
    private static final TurretGeometry TURRET_ON_BACK = new TurretGeometry(-6, 0, 180, +1);

    private static Pose robotAt(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ---------------- STEP 1: wrapDegrees ----------------

    @Test public void step1_wrapLeavesSmallAnglesAlone() {
        assertEquals(45, TurretGeometry.wrapDegrees(45), EPS);
        assertEquals(-90, TurretGeometry.wrapDegrees(-90), EPS);
        assertEquals(0, TurretGeometry.wrapDegrees(0), EPS);
    }

    @Test public void step1_wrapGoesAroundTheCircle() {
        assertEquals(-170, TurretGeometry.wrapDegrees(190), EPS);
        assertEquals(170, TurretGeometry.wrapDegrees(-190), EPS);
        assertEquals(0, TurretGeometry.wrapDegrees(360), EPS);
        assertEquals(10, TurretGeometry.wrapDegrees(730), EPS);   // two full turns plus 10
    }

    @Test public void step1_wrapPicksPlus180NotMinus180() {
        assertEquals(180, TurretGeometry.wrapDegrees(180), EPS);
        assertEquals(180, TurretGeometry.wrapDegrees(-180), EPS);
        assertEquals(180, TurretGeometry.wrapDegrees(540), EPS);
    }

    // ---------------- STEP 2: turretPosition ----------------

    @Test public void step2_noOffsetMeansTurretIsAtRobotCenter() {
        Pose turret = SIMPLE.turretPosition(robotAt(30, 40, 77));
        assertEquals(30, turret.getX(), EPS);
        assertEquals(40, turret.getY(), EPS);
    }

    @Test public void step2_offsetBehindRobotFacingPlusX() {
        // Robot faces +x, turret 6 in. behind it -> 6 in. toward -x.
        Pose turret = TURRET_ON_BACK.turretPosition(robotAt(50, 50, 0));
        assertEquals(44, turret.getX(), EPS);
        assertEquals(50, turret.getY(), EPS);
    }

    @Test public void step2_offsetRotatesWithTheRobot() {
        // Same robot, now facing +y. "Behind" is now toward -y.
        Pose turret = TURRET_ON_BACK.turretPosition(robotAt(50, 50, 90));
        assertEquals(50, turret.getX(), EPS);
        assertEquals(44, turret.getY(), EPS);
    }

    @Test public void step2_sidewaysOffset() {
        // Turret 4 in. to the robot's LEFT. Robot faces +x, so left is +y.
        TurretGeometry leftTurret = new TurretGeometry(0, 4, 0, +1);
        Pose turret = leftTurret.turretPosition(robotAt(10, 10, 0));
        assertEquals(10, turret.getX(), EPS);
        assertEquals(14, turret.getY(), EPS);
    }

    // ---------------- STEP 3: aimAngle ----------------

    @Test public void step3_targetStraightAhead() {
        assertEquals(0, SIMPLE.aimAngle(robotAt(0, 0, 0), 50, 0), DEG);
    }

    @Test public void step3_targetToTheLeftAndRight() {
        assertEquals(90, SIMPLE.aimAngle(robotAt(0, 0, 0), 0, 50), DEG);
        assertEquals(-90, SIMPLE.aimAngle(robotAt(0, 0, 0), 0, -50), DEG);
    }

    @Test public void step3_targetBehind() {
        assertEquals(180, SIMPLE.aimAngle(robotAt(0, 0, 0), -50, 0), DEG);
    }

    @Test public void step3_targetAt45Degrees() {
        assertEquals(45, SIMPLE.aimAngle(robotAt(0, 0, 0), 30, 30), DEG);
    }

    @Test public void step3_robotTurnedAway() {
        // Robot faces +y (90 deg). A target out on +x is now to its RIGHT.
        assertEquals(-90, SIMPLE.aimAngle(robotAt(0, 0, 90), 50, 0), DEG);
    }

    @Test public void step3_answerIsWrapped() {
        // Robot faces -170 deg; target is at field direction +170 deg.
        // That's 340 deg one way or -20 the other -- the answer must be -20.
        Pose robot = robotAt(0, 0, -170);
        double tx = 50 * Math.cos(Math.toRadians(170));
        double ty = 50 * Math.sin(Math.toRadians(170));
        assertEquals(-20, SIMPLE.aimAngle(robot, tx, ty), DEG);
    }

    @Test public void step3_offsetChangesTheAngle() {
        // Turret 6 in. to the robot's left; target dead ahead of the robot CENTER,
        // 30 in. away. From the turret, the target is a little to the RIGHT.
        TurretGeometry leftTurret = new TurretGeometry(0, 6, 0, +1);
        double expected = Math.toDegrees(Math.atan2(-6, 30));   // about -11.3
        assertEquals(expected, leftTurret.aimAngle(robotAt(0, 0, 0), 30, 0), DEG);
    }

    @Test public void step3_turretZeroFacesBackward() {
        // Turret 0 deg faces the robot's back. A target BEHIND the robot needs 0.
        assertEquals(0, TURRET_ON_BACK.aimAngle(robotAt(50, 50, 0), 0, 50), DEG);
        // A target AHEAD needs a half turn.
        assertEquals(180, TURRET_ON_BACK.aimAngle(robotAt(50, 50, 0), 100, 50), DEG);
    }

    @Test public void step3_clockwiseTurretFlipsTheSign() {
        TurretGeometry clockwise = new TurretGeometry(0, 0, 0, -1);
        assertEquals(-90, clockwise.aimAngle(robotAt(0, 0, 0), 0, 50), DEG);
    }

    // ---------------- STEP 4: distanceTo ----------------

    @Test public void step4_distanceFromCenter() {
        assertEquals(50, SIMPLE.distanceTo(robotAt(0, 0, 33), 30, 40), EPS);   // 3-4-5 triangle
    }

    @Test public void step4_distanceIsFromTheTurretNotTheRobot() {
        // Turret is 6 in. behind center; the target 30 in. ahead of center is 36 from the turret.
        assertEquals(36, TURRET_ON_BACK.distanceTo(robotAt(50, 50, 0), 80, 50), EPS);
    }

    // ---------------- STEP 5: turretFieldHeading ----------------

    @Test public void step5_pointingWhereWeAimed() {
        // Aim at a target, then ask which way the turret faces. It must face the target.
        TurretGeometry robot = new TurretGeometry(-6, 2, 180, -1);
        Pose pose = robotAt(40, 70, 25);
        double tx = 72, ty = 100;
        double turretDeg = robot.aimAngle(pose, tx, ty);
        Pose turret = robot.turretPosition(pose);
        double expected = Math.atan2(ty - turret.getY(), tx - turret.getX());
        double got = robot.turretFieldHeading(pose, turretDeg);
        assertEquals(0, TurretGeometry.wrapDegrees(Math.toDegrees(got - expected)), DEG);
    }

    // ---------------- STEP 6: robotPoseFromTurret ----------------

    @Test public void step6_roundTripGetsTheRobotBack() {
        // Go forward (robot -> turret) and back (turret -> robot) for many
        // robots and poses. If ANY sign is wrong in either direction, this fails.
        TurretGeometry[] robots = {
                SIMPLE, TURRET_ON_BACK,
                new TurretGeometry(-6, 2, 180, -1),
                new TurretGeometry(3, -4, 90, +1),
        };
        Pose[] poses = { robotAt(10, 20, 0), robotAt(72, 72, 135), robotAt(100, 30, -60) };
        double[] turretAngles = { 0, 37, -120 };

        for (TurretGeometry robot : robots) {
            for (Pose pose : poses) {
                for (double turretDeg : turretAngles) {
                    Pose turret = robot.turretPosition(pose);
                    double heading = robot.turretFieldHeading(pose, turretDeg);
                    Pose back = robot.robotPoseFromTurret(turret.getX(), turret.getY(), heading, turretDeg);
                    String where = "offset (" + robot.offsetX + "," + robot.offsetY + "), turret " + turretDeg;
                    assertEquals(where, pose.getX(), back.getX(), EPS);
                    assertEquals(where, pose.getY(), back.getY(), EPS);
                    assertEquals(where, 0, TurretGeometry.wrapDegrees(
                            Math.toDegrees(back.getHeading() - pose.getHeading())), DEG);
                }
            }
        }
    }

    // ---------------- Already done for you ----------------

    @Test public void notConfiguredUntilMeasured() {
        assertFalse(new TurretGeometry(Double.NaN, 0, 0, 1).isConfigured());
        assertFalse(new TurretGeometry(0, 0, 0, 0.5).isConfigured());   // sign must be exactly +1 or -1
        assertTrue(SIMPLE.isConfigured());
    }
}
