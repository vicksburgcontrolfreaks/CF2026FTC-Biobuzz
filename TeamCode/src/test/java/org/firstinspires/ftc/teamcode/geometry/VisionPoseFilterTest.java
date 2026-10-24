package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.geometry.VisionPoseFilter.Verdict;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/*
 * Tests for VisionPoseFilter (STEPS 7-8). Laptop only:
 *     gradlew :TeamCode:testDebugUnitTest
 *
 * The tests use the default thresholds in VisionPoseFilter. If you tune those
 * on the robot later, some of the numbers here may need to change with them.
 */
public class VisionPoseFilterTest {

    private static final double EPS = 1e-6;

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private static final Pose ODO = at(50, 50, 90);
    private static final Pose CLOSE = at(51, 49, 92);   // a believable fix: an inch or two off

    // A fix that passes every rule. Each test below breaks exactly ONE thing.
    private static Verdict goodExcept(boolean settled, double speed, double turn,
                                      double age, double range, Pose vision) {
        return VisionPoseFilter.check(settled, speed, turn, age, range, ODO, vision);
    }

    // ---------------- STEP 7: check ----------------

    @Test public void step7_goodFixIsAccepted() {
        assertEquals(Verdict.ACCEPT, goodExcept(true, 0, 0, 50, 40, CLOSE));
    }

    @Test public void step7_turretMoving() {
        assertEquals(Verdict.TURRET_MOVING, goodExcept(false, 0, 0, 50, 40, CLOSE));
    }

    @Test public void step7_robotDrivingOrTurning() {
        assertEquals(Verdict.ROBOT_MOVING, goodExcept(true, 12, 0, 50, 40, CLOSE));
        assertEquals(Verdict.ROBOT_MOVING, goodExcept(true, 0, 45, 50, 40, CLOSE));
        assertEquals(Verdict.ROBOT_MOVING, goodExcept(true, 0, -45, 50, 40, CLOSE));   // turning right counts too
    }

    @Test public void step7_staleFrame() {
        assertEquals(Verdict.FRAME_TOO_OLD, goodExcept(true, 0, 0, 400, 40, CLOSE));
    }

    @Test public void step7_tagTooCloseOrTooFar() {
        assertEquals(Verdict.TAG_OUT_OF_RANGE, goodExcept(true, 0, 0, 50, 6, CLOSE));
        assertEquals(Verdict.TAG_OUT_OF_RANGE, goodExcept(true, 0, 0, 50, 120, CLOSE));
    }

    @Test public void step7_fixTooFarFromOdometry() {
        assertEquals(Verdict.TOO_FAR_FROM_ODOMETRY, goodExcept(true, 0, 0, 50, 40, at(80, 50, 90)));
        assertEquals(Verdict.TOO_FAR_FROM_ODOMETRY, goodExcept(true, 0, 0, 50, 40, at(50, 50, 150)));
    }

    @Test public void step7_headingsAcrossZeroAreClose() {
        // Odometry says 359 deg, vision says 1 deg: only 2 deg apart. Accept it.
        Verdict v = VisionPoseFilter.check(true, 0, 0, 50, 40, at(50, 50, 359), at(50, 50, 1));
        assertEquals(Verdict.ACCEPT, v);
    }

    @Test public void step7_firstFailingRuleWins() {
        // Turret moving AND robot moving AND stale: report the first rule, TURRET_MOVING.
        assertEquals(Verdict.TURRET_MOVING, goodExcept(false, 12, 45, 400, 40, CLOSE));
    }

    // ---------------- STEP 8: blend ----------------

    @Test public void step8_fractionZeroAndOne() {
        Pose stay = VisionPoseFilter.blend(at(10, 20, 30), at(20, 40, 50), 0);
        assertEquals(10, stay.getX(), EPS);
        assertEquals(20, stay.getY(), EPS);
        Pose jump = VisionPoseFilter.blend(at(10, 20, 30), at(20, 40, 50), 1);
        assertEquals(20, jump.getX(), EPS);
        assertEquals(40, jump.getY(), EPS);
        assertEquals(50, Math.toDegrees(jump.getHeading()), EPS);
    }

    @Test public void step8_tenPercent() {
        Pose p = VisionPoseFilter.blend(at(10, 20, 30), at(20, 40, 50), 0.1);
        assertEquals(11, p.getX(), EPS);
        assertEquals(22, p.getY(), EPS);
        assertEquals(32, Math.toDegrees(p.getHeading()), EPS);
    }

    @Test public void step8_headingBlendsTheShortWay() {
        // Halfway from 350 to 10 is 0 (or 360), not 180.
        Pose p = VisionPoseFilter.blend(at(0, 0, 350), at(0, 0, 10), 0.5);
        assertEquals(0, TurretGeometry.wrapDegrees(Math.toDegrees(p.getHeading())), EPS);
    }
}
