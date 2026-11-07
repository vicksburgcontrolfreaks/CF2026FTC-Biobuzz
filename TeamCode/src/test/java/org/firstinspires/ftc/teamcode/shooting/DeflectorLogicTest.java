package org.firstinspires.ftc.teamcode.shooting;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/* DeflectorLogic on the laptop:  gradlew :TeamCode:testDebugUnitTest */
public class DeflectorLogicTest {

    private static final double EPS = 1e-9;

    @Before
    public void setUp() {
        DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG = 2687.0 / 90.0;
        DeflectorLogic.POWER_SIGN = 1.0;
        DeflectorLogic.POSITIONS = new double[] {0, 70, 75, 80, 90};
        DeflectorLogic.MIN_ANGLE = -2;
        DeflectorLogic.MAX_ANGLE = 92;
        DeflectorLogic.kP = 0.08;
        DeflectorLogic.MAX_POWER = 0.8;
        DeflectorLogic.MIN_POWER = 0.05;
        DeflectorLogic.TOLERANCE_DEG = 0.5;
    }

    // ---- unwrap ----

    @Test public void unwrap_normalStep() {
        assertEquals(10, DeflectorLogic.unwrapStep(100, 110), EPS);
        assertEquals(-10, DeflectorLogic.unwrapStep(110, 100), EPS);
    }

    @Test public void unwrap_crossingTheTopGoingUp() {
        assertEquals(15, DeflectorLogic.unwrapStep(355, 10), EPS);
    }

    @Test public void unwrap_crossingTheTopGoingDown() {
        assertEquals(-15, DeflectorLogic.unwrapStep(10, 355), EPS);
    }

    @Test public void unwrap_sevenAndAHalfTurns_addsUpTo90Degrees() {
        // Walk 2687 servo degrees in 12-degree steps, raw wrapping every turn.
        double raw = 0, total = 0, walked = 0;
        while (walked < 2687) {
            double step = Math.min(12, 2687 - walked);
            double next = (raw + step) % 360;
            total += DeflectorLogic.unwrapStep(raw, next);
            raw = next;
            walked += step;
        }
        assertEquals(90, DeflectorLogic.toDeflectorDeg(total, 0), 1e-6);
    }

    // ---- angle ----

    @Test public void angle_isRelativeToTheZero() {
        double atZero = 5000;
        assertEquals(0, DeflectorLogic.toDeflectorDeg(5000, atZero), EPS);
        assertEquals(75, DeflectorLogic.toDeflectorDeg(5000 + 75 * 2687.0 / 90, atZero), 1e-6);
    }

    // ---- control ----

    @Test public void power_zeroInsideTolerance() {
        assertEquals(0, DeflectorLogic.power(74.7, 75), EPS);
    }

    @Test public void power_bigErrorIsCappedAndPointsTheRightWay() {
        assertEquals(0.8, DeflectorLogic.power(0, 90), EPS);
        assertEquals(-0.8, DeflectorLogic.power(90, 0), EPS);
    }

    @Test public void power_smallErrorGetsAtLeastMinPower() {
        // 0.6 deg off: kP gives 0.048, under MIN_POWER -> bumped to 0.05.
        assertEquals(0.05, DeflectorLogic.power(74.4, 75), 1e-9);
    }

    @Test public void power_neverPushesPastTheSoftLimits() {
        assertEquals(0, DeflectorLogic.power(93, 200), EPS);
        assertEquals(0, DeflectorLogic.power(-3, -100), EPS);
        // ...but it may still come BACK from past a limit.
        assertTrue(DeflectorLogic.power(93, 90) < 0);
    }

    @Test public void power_signFlipsWithPowerSign() {
        DeflectorLogic.POWER_SIGN = -1;
        assertEquals(-0.8, DeflectorLogic.power(0, 90), EPS);
    }

    @Test public void nearestIndex_picksClosestPosition() {
        assertEquals(0, DeflectorLogic.nearestIndex(10));
        assertEquals(2, DeflectorLogic.nearestIndex(76));
        assertEquals(4, DeflectorLogic.nearestIndex(100));
    }
}
