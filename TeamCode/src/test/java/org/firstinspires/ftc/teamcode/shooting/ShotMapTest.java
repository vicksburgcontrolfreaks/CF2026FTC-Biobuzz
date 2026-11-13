package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.shooting.ShotMap.Bracket;
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Outcome;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/*
 * ShotMap on a laptop. The bracket tests play a pretend shooter whose hit band
 * we choose, follow suggest() the way an operator would, and check the band
 * that comes out -- and how many balls it took.
 */
public class ShotMapTest {

    private static final double EPS = 1e-6;

    /** A shooter that hits between lo and hi RPM, inclusive, every time. */
    private static Bracket.Result shoot(double rpm, double lo, double hi) {
        return rpm < lo ? Bracket.Result.LOW : rpm > hi ? Bracket.Result.HIGH : Bracket.Result.HIT;
    }

    /** Follow suggest() from `start` until done. Returns the number of balls. */
    private static int hunt(Bracket b, double start, double lo, double hi,
                            double step, double resolution, int maxBalls) {
        double rpm = start;
        for (int ball = 1; ball <= maxBalls; ball++) {
            b.record(rpm, shoot(rpm, lo, hi));
            if (b.isDone(resolution)) return ball;
            rpm = b.suggest(rpm, step, resolution);
        }
        return maxBalls + 1;
    }

    // ---- Bracket: the search -------------------------------------------------

    @Test
    public void findsTheBandFromBelow() {
        Bracket b = new Bracket();
        int balls = hunt(b, 1800, 2250, 2350, 100, 10, 40);
        assertTrue("took " + balls + " balls", balls <= 16);
        assertEquals(2300, b.centre(), 10);
        assertEquals(100, b.width(), 20);
        assertFalse(b.isNoisy());
    }

    @Test
    public void findsTheBandFromAbove() {
        Bracket b = new Bracket();
        int balls = hunt(b, 2900, 2250, 2350, 100, 10, 40);
        assertTrue("took " + balls + " balls", balls <= 18);
        assertEquals(2300, b.centre(), 10);
    }

    @Test
    public void findsTheBandStartingInsideIt() {
        Bracket b = new Bracket();
        int balls = hunt(b, 2280, 2250, 2350, 100, 10, 40);
        assertTrue("took " + balls + " balls", balls <= 14);
        assertEquals(2300, b.centre(), 10);
    }

    @Test
    public void aNarrowBandBetweenBigStepsIsStillFound() {
        // Step 100 jumps straight over a 30-RPM band; the short/long split finds it.
        Bracket b = new Bracket();
        int balls = hunt(b, 2000, 2310, 2340, 100, 5, 40);
        assertTrue("took " + balls + " balls", balls <= 20);
        assertEquals(2325, b.centre(), 6);
    }

    @Test
    public void exploresOutwardFromAHitBeforeSplitting() {
        Bracket b = new Bracket();
        b.record(2300, Bracket.Result.HIT);
        double next = b.suggest(2300, 100, 10);
        assertTrue(next == 2200 || next == 2400);
    }

    @Test
    public void noShotsMeansStayPut() {
        assertEquals(2300, new Bracket().suggest(2300, 100, 10), EPS);
        assertTrue(Double.isNaN(new Bracket().centre()));
    }

    @Test
    public void edgesAreHalfwayAcrossTheGaps() {
        Bracket b = new Bracket();
        b.record(2200, Bracket.Result.LOW);
        b.record(2260, Bracket.Result.HIT);
        b.record(2340, Bracket.Result.HIT);
        b.record(2400, Bracket.Result.HIGH);
        assertEquals(2230, b.lowEdge(), EPS);
        assertEquals(2370, b.highEdge(), EPS);
        assertEquals(2300, b.centre(), EPS);
        assertEquals(140, b.width(), EPS);
    }

    @Test
    public void aShortAboveAHitIsNoisy() {
        Bracket b = new Bracket();
        b.record(2300, Bracket.Result.HIT);
        b.record(2350, Bracket.Result.LOW);
        assertTrue(b.isNoisy());
        assertFalse(b.isDone(1000));
    }

    // ---- Outcome mapping -------------------------------------------------------

    @Test
    public void outcomesFeedTheRightSearch() {
        assertEquals(Bracket.Result.LOW, Outcome.SHORT.forRpm());
        assertEquals(Bracket.Result.HIGH, Outcome.LONG.forRpm());
        assertNull(Outcome.LEFT.forRpm());
        // Trim + = turret further left, so a ball that went LEFT had too much.
        assertEquals(Bracket.Result.HIGH, Outcome.LEFT.forAim());
        assertEquals(Bracket.Result.LOW, Outcome.RIGHT.forAim());
        assertNull(Outcome.SHORT.forAim());
        assertEquals(Bracket.Result.HIT, Outcome.HIT.forRpm());
        assertEquals(Bracket.Result.HIT, Outcome.HIT.forAim());
    }

    // ---- fit -------------------------------------------------------------------

    @Test
    public void fitRecoversTheFlywheelOffsetTerms() {
        // rpm = 1500 + 12*d + 40*cos(t) - 70*sin(t), measured at 3 distances x 5 angles.
        double[] ds = {40, 60, 80};
        double[] ts = {-90, -45, 0, 45, 90};
        double[][] rows = new double[ds.length * ts.length][];
        double[] y = new double[rows.length];
        int i = 0;
        for (double d : ds) {
            for (double t : ts) {
                rows[i] = ShotMap.rpmRow(d, t);
                double r = Math.toRadians(t);
                y[i++] = 1500 + 12 * d + 40 * Math.cos(r) - 70 * Math.sin(r);
            }
        }
        double[] c = ShotMap.fit(rows, y);
        assertNotNull(c);
        assertEquals(1500, c[0], 1e-6);
        assertEquals(12, c[1], 1e-6);
        assertEquals(40, c[2], 1e-6);
        assertEquals(-70, c[3], 1e-6);
    }

    @Test
    public void fitRefusesWhenEverySpotIsAtOneTurretAngle() {
        // cos and sin are constant -> can't be told apart from the constant term.
        double[][] rows = {ShotMap.rpmRow(40, 0), ShotMap.rpmRow(60, 0),
                ShotMap.rpmRow(80, 0), ShotMap.rpmRow(100, 0)};
        assertNull(ShotMap.fit(rows, new double[]{2000, 2200, 2400, 2600}));
    }

    @Test
    public void fitRefusesTooFewRows() {
        assertNull(ShotMap.fit(new double[][]{ShotMap.aimRow(0), ShotMap.aimRow(90)},
                new double[]{1, 2}));
    }

    @Test
    public void distinctAnglesBinsNearbyOnes() {
        assertEquals(3, ShotMap.distinctAngles(new double[]{-1, 2, 44, 46, 90}, 10));
    }

    // ---- CSV ---------------------------------------------------------------------

    @Test
    public void csvRowsMatchTheirHeaders() {
        String shot = ShotMap.shotRow(1.5, "6494", 2, 7, "red, audience wall", 33, 9, 0, 61.2,
                23.4, 2300, 2295, 2301, 1, 85, 0.5, 67, true, Outcome.SHORT);
        assertEquals(ShotMap.SHOT_HEADER.split(",").length, shot.split(",", -1).length);
        assertTrue("commas in names must not split columns", shot.contains("red; audience wall"));

        Bracket b = new Bracket();
        b.record(2300, Bracket.Result.HIT);
        String spot = ShotMap.spotRow("6494", 2, 61.2, 23.4, 5, b, new Bracket());
        assertEquals(ShotMap.SPOT_HEADER.split(",").length, spot.split(",", -1).length);
    }
}
