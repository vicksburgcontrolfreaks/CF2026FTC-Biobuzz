package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * Tests for OdometryAim. Run with:  gradlew :TeamCode:testDebugUnitTest
 *
 * Limits here are our real ones (-140 / +140), hard-coded so changing
 * Turret.MIN_ANGLE later doesn't silently change what these tests check.
 * Dead zone = 140..180..-140, middle at 180.
 */
public class OdometryAimTest {

    private static final double EPS = 1e-6;
    private static final double MIN = -140, MAX = 140, HYST = 15;

    private static final TurretGeometry SIMPLE = new TurretGeometry(0, 0, 0, +1);

    private static double cmd(double desired, double current) {
        return OdometryAim.reachableCommand(desired, current, MIN, MAX, HYST);
    }

    // ---------------- reachable: pass straight through ----------------

    @Test public void reachableAnglesPassThrough() {
        assertEquals(0, cmd(0, 0), EPS);
        assertEquals(90, cmd(90, -120), EPS);
        assertEquals(-135, cmd(-135, 100), EPS);
        assertEquals(140, cmd(140, 0), EPS);    // exactly on the limit counts
        assertEquals(-140, cmd(-140, 0), EPS);
    }

    @Test public void unwrappedInputIsWrappedFirst() {
        assertEquals(10, cmd(370, 0), EPS);
        assertEquals(-90, cmd(270, 0), EPS);
    }

    // ---------------- dead zone: park at a limit ----------------

    @Test public void deadZoneParksOnTheSideTheTurretIsAlreadyOn() {
        // Target just past +140; turret on the + side -> stay at +140.
        assertEquals(MAX, cmd(150, 120), EPS);
        // Same target, turret on the - side, target only 30 deg from middle on
        // the + side (more than HYST) -> come across to +140.
        assertEquals(MAX, cmd(150, -120), EPS);
        // Mirror image.
        assertEquals(MIN, cmd(-150, -120), EPS);
        assertEquals(MIN, cmd(-150, 120), EPS);
    }

    @Test public void targetCrossingTheMiddleDoesNotFlipTheTurret() {
        // Turret parked at +140. Robot spins so the target walks 170 -> 180 -> -170.
        // Without hysteresis this is a 280 deg swing at 180.
        assertEquals(MAX, cmd(170, MAX), EPS);
        assertEquals(MAX, cmd(180, MAX), EPS);
        assertEquals(MAX, cmd(-170, MAX), EPS);   // 10 past middle < HYST: stay
        assertEquals(MAX, cmd(-166, MAX), EPS);   // 14 past middle: still stay
        assertEquals(MIN, cmd(-160, MAX), EPS);   // 20 past middle: now switch
    }

    @Test public void hysteresisWorksFromTheMinSideToo() {
        assertEquals(MIN, cmd(-170, MIN), EPS);
        assertEquals(MIN, cmd(170, MIN), EPS);    // 10 past middle on the + side: stay
        assertEquals(MAX, cmd(160, MIN), EPS);    // 20 past: switch
    }

    @Test public void zeroHysteresisMeansNearestLimit() {
        assertEquals(MAX, OdometryAim.reachableCommand(175, -100, MIN, MAX, 0), EPS);
        assertEquals(MIN, OdometryAim.reachableCommand(-175, 100, MIN, MAX, 0), EPS);
    }

    @Test public void lopsidedLimitsPutTheMiddleInTheRightPlace() {
        // Travel -100..+160: dead zone 160..260 (= -100), middle at 210 (= -150).
        // Target at -140 is 10 deg past the middle toward -100.
        assertEquals(160, OdometryAim.reachableCommand(-140, 150, -100, 160, 15), EPS);
        assertEquals(-100, OdometryAim.reachableCommand(-140, 150, -100, 160, 5), EPS);
    }

    // ---------------- solve(): the whole chain ----------------

    @Test public void solveReachableTarget() {
        Pose robot = new Pose(72, 72, Math.toRadians(0));
        OdometryAim.Solution s = OdometryAim.solve(SIMPLE, robot, 72, 100, 0, MIN, MAX, HYST);
        assertEquals(90, s.desiredDeg, EPS);
        assertEquals(90, s.commandDeg, EPS);
        assertTrue(s.reachable);
        assertEquals(0, s.outOfReachDeg, EPS);
        assertEquals(28, s.distance, EPS);
    }

    @Test public void solveTargetBehindIsOutOfReach() {
        // Target straight behind: desired 180, the middle of the dead zone.
        Pose robot = new Pose(72, 72, Math.toRadians(0));
        OdometryAim.Solution s = OdometryAim.solve(SIMPLE, robot, 40, 72, 30, MIN, MAX, HYST);
        assertEquals(180, s.desiredDeg, EPS);
        assertEquals(MAX, s.commandDeg, EPS);    // turret was on the + side
        assertFalse(s.reachable);
        assertEquals(40, s.outOfReachDeg, EPS);  // 180 - 140
        assertEquals(32, s.distance, EPS);
    }

    @Test public void solveUsesTheTurretOffset() {
        // Turret 6 in. left of center. Target 6 in. left and 20 ahead of the
        // TURRET -> straight ahead for the turret, though not for the robot center.
        TurretGeometry leftTurret = new TurretGeometry(0, 6, 0, +1);
        Pose robot = new Pose(72, 72, Math.toRadians(90));   // facing +y; left = -x
        OdometryAim.Solution s = OdometryAim.solve(leftTurret, robot, 66, 92, 0, MIN, MAX, HYST);
        assertEquals(0, s.desiredDeg, 1e-3);
        assertEquals(20, s.distance, EPS);
    }
}
