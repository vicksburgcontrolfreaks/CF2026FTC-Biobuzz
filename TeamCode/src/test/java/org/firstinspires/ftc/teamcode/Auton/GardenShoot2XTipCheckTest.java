package org.firstinspires.ftc.teamcode.Auton;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;

/**
 * Garden Shoot 2X Tip Check (Shane, 2026-11-15): Garden Shoot 2X, except the
 * second volley only fires if the hive has NOT tipped.
 *
 * The whole promise is "exactly Garden Shoot 2X apart from that one step", so
 * that is what these pin: same start, same steps with the same points, speeds,
 * dwells, timeouts and fire times, same park marker -- and the one step that
 * differs is the second volley. GardenShoot2XTest already checks the paths and
 * clearances of those identical steps.
 */
public class GardenShoot2XTipCheckTest {

    private static final double EPS = 1e-9;
    private static final int SECOND_VOLLEY = 3;

    @Test public void identicalToGardenShoot2XExceptTheSecondVolley() {
        for (Routes.Park p : Routes.Park.values()) {
            Route plain = Routes.gardenShoot2X(p);
            Route check = Routes.gardenShoot2XTipCheck(p);

            assertEquals(p + " start x", plain.start.getX(), check.start.getX(), EPS);
            assertEquals(p + " start y", plain.start.getY(), check.start.getY(), EPS);
            assertEquals(p + " start heading", plain.start.getHeading(), check.start.getHeading(), EPS);
            assertEquals(p + " step count", plain.steps.size(), check.steps.size());
            assertEquals(p + " park marker", plain.parkFrom, check.parkFrom);

            for (int i = 0; i < plain.steps.size(); i++) {
                Route.Step a = plain.steps.get(i);
                Route.Step b = check.steps.get(i);
                String at = p + " step " + (i + 1);
                if (i == SECOND_VOLLEY) {
                    assertEquals(at, Route.Action.LAUNCH_HERE, a.action);
                    assertEquals(at, Route.Action.LAUNCH_HERE_IF_UP, b.action);
                } else {
                    assertEquals(at, a.action, b.action);
                }
                if (a.target == null) {
                    assertNull(at, b.target);
                } else {
                    assertEquals(at + " x", a.target.getX(), b.target.getX(), EPS);
                    assertEquals(at + " y", a.target.getY(), b.target.getY(), EPS);
                    assertEquals(at + " heading", a.target.getHeading(), b.target.getHeading(), EPS);
                }
                assertEquals(at + " fireSec", a.fireSec, b.fireSec, EPS);
                assertEquals(at + " cutForPark", a.cutForPark, b.cutForPark);
                assertEquals(at + " speed", a.speed, b.speed, EPS);
                assertEquals(at + " dwell", a.dwellSec, b.dwellSec, EPS);
                assertEquals(at + " timeout", a.timeoutSec, b.timeoutSec, EPS);
            }
        }
    }

    /** The checked volley: 4 s like the plain one, no path, and cut for the park. */
    @Test public void secondVolleyIsTheFourSecondOneAndRespectsThePark() {
        Route.Step s = Routes.gardenShoot2XTipCheck(Routes.Park.GARDEN_SIDE).steps.get(SECOND_VOLLEY);
        assertEquals(Route.Action.LAUNCH_HERE_IF_UP, s.action);
        assertEquals(Routes.SHOOT_2X_FIRE_SEC, s.fireSec, EPS);
        assertEquals(4.0, s.fireSec, EPS);
        assertNull(s.target);
        assertTrue(s.cutForPark);
    }

    /** The FIRST volley is never checked -- the preloads always go. */
    @Test public void firstVolleyIsUnconditional() {
        Route r = Routes.gardenShoot2XTipCheck(Routes.Park.GARDEN_SIDE);
        assertEquals(Route.Action.LAUNCH_HERE, r.steps.get(0).action);
        assertEquals(Routes.SHOOT_2X_FIRST_FIRE_SEC, r.steps.get(0).fireSec, EPS);
    }

    /** Skipping the volley goes straight into the park: it is the step before parkFrom. */
    @Test public void skippedVolleyLeadsStraightIntoThePark() {
        Route r = Routes.gardenShoot2XTipCheck(Routes.Park.GARDEN_SIDE);
        assertEquals(SECOND_VOLLEY + 1, r.parkFrom);
        assertTrue(r.has(Route.Action.LAUNCH_HERE_IF_UP));
        assertFalse(Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE).has(Route.Action.LAUNCH_HERE_IF_UP));
    }
}
