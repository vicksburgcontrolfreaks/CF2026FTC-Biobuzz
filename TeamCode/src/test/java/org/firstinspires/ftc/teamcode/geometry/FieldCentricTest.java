package org.firstinspires.ftc.teamcode.geometry;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/*
 * FieldCentric on the laptop:  gradlew :TeamCode:testDebugUnitTest
 *
 * Each test is one picture: where the robot faces, which way the stick goes,
 * and which way the ROBOT should move (forward / right of its own front).
 */
public class FieldCentricTest {

    private static final double EPS = 1e-9;
    private static final double RED_AWAY = 0.0;        // red drivers look toward +x
    private static final double BLUE_AWAY = Math.PI;   // blue drivers look toward -x

    private static void assertRobot(double[] got, double forward, double right) {
        assertEquals("robot forward", forward, got[0], EPS);
        assertEquals("robot right", right, got[1], EPS);
    }

    @Test
    public void robotFacingAway_stickMapsStraightThrough() {
        assertRobot(FieldCentric.toRobot(1, 0, RED_AWAY, RED_AWAY), 1, 0);
        assertRobot(FieldCentric.toRobot(0, 1, RED_AWAY, RED_AWAY), 0, 1);
    }

    @Test
    public void robotFacingTheDriver_everythingFlips() {
        // Robot turned around to face the red driver: stick "away" = robot BACKWARD.
        assertRobot(FieldCentric.toRobot(1, 0, Math.PI, RED_AWAY), -1, 0);
        assertRobot(FieldCentric.toRobot(0, 1, Math.PI, RED_AWAY), 0, -1);
    }

    @Test
    public void robotTurnedLeft_stickAwayMeansStrafeRight() {
        // Robot turned 90 deg left (CCW) of "away": away is now off its RIGHT side.
        assertRobot(FieldCentric.toRobot(1, 0, Math.PI / 2, RED_AWAY), 0, 1);
        // Driver's right = field -y. The robot faces +y, so that's robot BACKWARD.
        assertRobot(FieldCentric.toRobot(0, 1, Math.PI / 2, RED_AWAY), -1, 0);
    }

    @Test
    public void blueAlliance_robotFacingAwayFromBlueDrivers_isStraightThrough() {
        // Facing -x (heading 180) is "away" for blue.
        assertRobot(FieldCentric.toRobot(1, 0, Math.PI, BLUE_AWAY), 1, 0);
        assertRobot(FieldCentric.toRobot(0, 1, Math.PI, BLUE_AWAY), 0, 1);
    }

    @Test
    public void blueAlliance_robotFacingPlusX_isTowardBlueDrivers() {
        assertRobot(FieldCentric.toRobot(1, 0, 0.0, BLUE_AWAY), -1, 0);
    }

    @Test
    public void matchesTheOldRedOnlyFormula() {
        // The DriveTeleOp formula robot-tested on 10/27 (heading 0 at INIT).
        double f = 0.6, r = -0.3;
        for (double h = -3; h <= 3; h += 0.25) {
            double oldRotY = r * Math.sin(-h) + f * Math.cos(-h);
            double oldRotX = r * Math.cos(-h) - f * Math.sin(-h);
            assertRobot(FieldCentric.toRobot(f, r, h, RED_AWAY), oldRotY, oldRotX);
        }
    }
}
