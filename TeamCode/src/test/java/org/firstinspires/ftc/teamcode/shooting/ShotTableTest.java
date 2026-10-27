package org.firstinspires.ftc.teamcode.shooting;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * Tests for ShotTable. Run with:  gradlew :TeamCode:testDebugUnitTest
 * The example table is made up -- the real one is ShotTable.DISTANCES / RPMS.
 */
public class ShotTableTest {

    private static final double EPS = 1e-9;

    private static final ShotTable EXAMPLE = new ShotTable(
            new double[] { 36, 60, 84, 108 },
            new double[] { 2100, 2400, 2800, 3300 });

    @Test public void exactRowsComeBackExactly() {
        assertEquals(2100, EXAMPLE.lookup(36), EPS);
        assertEquals(2400, EXAMPLE.lookup(60), EPS);
        assertEquals(3300, EXAMPLE.lookup(108), EPS);
    }

    @Test public void betweenRowsIsAStraightLine() {
        assertEquals(2600, EXAMPLE.lookup(72), EPS);    // halfway 60..84
        assertEquals(2175, EXAMPLE.lookup(42), EPS);    // a quarter of 36..60
        assertEquals(3175, EXAMPLE.lookup(102), EPS);   // three quarters of 84..108
    }

    @Test public void offTheEndsHoldsTheEndValueAndSaysSo() {
        assertEquals(2100, EXAMPLE.lookup(10), EPS);
        assertEquals(3300, EXAMPLE.lookup(140), EPS);
        assertFalse(EXAMPLE.inRange(35.9));
        assertFalse(EXAMPLE.inRange(108.1));
        assertTrue(EXAMPLE.inRange(36));
        assertTrue(EXAMPLE.inRange(108));
        assertTrue(EXAMPLE.inRange(70));
    }

    @Test public void emptyTableIsNotConfigured() {
        ShotTable empty = new ShotTable(new double[] {}, new double[] {});
        assertFalse(empty.isConfigured());
        assertFalse(empty.inRange(60));
        assertTrue(Double.isNaN(empty.lookup(60)));
    }

    @Test public void oneRowTableIsThatValueEverywhere() {
        ShotTable one = new ShotTable(new double[] { 60 }, new double[] { 2400 });
        assertEquals(2400, one.lookup(20), EPS);
        assertEquals(2400, one.lookup(100), EPS);
        assertTrue(one.inRange(60));
        assertFalse(one.inRange(61));
    }

    @Test public void editingTheArraysLaterDoesNotChangeABuiltTable() {
        double[] d = { 36, 60 };
        double[] v = { 2100, 2400 };
        ShotTable t = new ShotTable(d, v);
        v[1] = 9999;
        assertEquals(2400, t.lookup(60), EPS);
    }

    @Test(expected = IllegalArgumentException.class)
    public void mismatchedLengthsAreRejected() {
        new ShotTable(new double[] { 36, 60 }, new double[] { 2100 });
    }

    @Test(expected = IllegalArgumentException.class)
    public void unsortedDistancesAreRejected() {
        new ShotTable(new double[] { 60, 36 }, new double[] { 2400, 2100 });
    }

    @Test(expected = IllegalArgumentException.class)
    public void repeatedDistanceIsRejected() {
        new ShotTable(new double[] { 36, 36 }, new double[] { 2100, 2200 });
    }

    @Test(expected = IllegalArgumentException.class)
    public void nanIsRejected() {
        new ShotTable(new double[] { 36, Double.NaN }, new double[] { 2100, 2200 });
    }

    @Test public void robotTableStartsEmpty() {
        // Flip this test when real rows go in -- it's here so nobody ships
        // made-up RPMs by accident.
        assertFalse(ShotTable.forRobot().isConfigured());
    }
}
