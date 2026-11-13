package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.shooting.ShotMap.Fit;
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Ladder;
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Outcome;
import org.junit.Test;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/*
 * ShotMap on a laptop.
 *
 * The ladder tests play a pretend shooter with RANDOM scatter: each ball lands
 * as if shot at (rpm + noise), and scores if that lands inside the hit band.
 * That is the situation Shane asked about -- balls and the mechanism are a bit
 * random -- and the old extremes-only search fell apart on it. These follow
 * next() the way the op mode does, over many seeded runs, and check the answer
 * and the ball count.
 */
public class ShotMapTest {

    private static final double EPS = 1e-6;

    /** Hit band [lo, hi] RPM, Gaussian scatter sigma RPM per ball. */
    private static Ladder.Result shoot(Random rnd, double rpm, double lo, double hi, double sigma) {
        double eff = rpm + rnd.nextGaussian() * sigma;
        return eff < lo ? Ladder.Result.LOW : eff > hi ? Ladder.Result.HIGH : Ladder.Result.HIT;
    }

    private static char letter(Ladder.Result r) {
        return r == Ladder.Result.LOW ? 'S' : r == Ladder.Result.HIGH ? 'L' : 'H';
    }

    /** Follow next() until done or out of balls. Returns balls used. */
    private static int run(Ladder ladder, Random rnd, double start, double lo, double hi,
                           double sigma, int maxBalls) {
        double rpm = start;
        for (int ball = 1; ball <= maxBalls; ball++) {
            Ladder.Result r = shoot(rnd, rpm, lo, hi, sigma);
            ladder.record(rpm, r, letter(r));
            if (ladder.isDone()) return ball;
            rpm = ladder.next(rpm);
        }
        return maxBalls + 1;
    }

    // ---- The question Shane asked: random balls -------------------------------

    @Test
    public void findsTheCentreThroughRandomScatter() {
        // Band 1240-1270 (centre 1255), 15 RPM of scatter per ball -- about as
        // wide as the band. 25 RPM rungs, 4 balls each, starting off-centre.
        int runs = 200, within15 = 0, unfinished = 0;
        int[] balls = new int[runs];
        for (int seed = 0; seed < runs; seed++) {
            Ladder ladder = new Ladder(25, 4, true);
            balls[seed] = run(ladder, new Random(seed), 1200, 1240, 1270, 15, 60);
            if (balls[seed] > 60) unfinished++;
            double b = ladder.balance();
            if (!Double.isNaN(b) && Math.abs(b - 1255) <= 15) within15++;
        }
        Arrays.sort(balls);
        int median = balls[runs / 2];
        System.out.printf("ladder: within 15 RPM %d/%d, balls median %d, 90th %d, unfinished %d%n",
                within15, runs, median, balls[(int) (runs * 0.9)], unfinished);
        assertTrue("within 15 RPM in only " + within15 + "/" + runs, within15 >= 0.9 * runs);
        assertTrue("median balls " + median, median <= 28);
        assertTrue("unfinished " + unfinished, unfinished <= 0.02 * runs);
    }

    @Test
    public void oneBadBallDoesNotMoveTheAnswer() {
        // The case that broke the old version: one SHORT inside a run of hits.
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1225, "SSSH");
        rec(l, 1250, "HHSH");      // the stray short
        rec(l, 1275, "LLHL");
        assertTrue(l.isBounded());
        assertFalse(l.isNoisy());
        // 1225 = -0.75, 1250 = -0.25, 1275 = +0.75: crosses 0 a quarter of the way up.
        assertEquals(1250 + 25 * 0.25, l.balance(), EPS);
        assertEquals(0.75, l.bestHitRate(), EPS);
    }

    @Test
    public void balanceSitsInTheMiddleOfAPlateauOfHits() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1200, "SSSS");
        rec(l, 1225, "HHHH");
        rec(l, 1250, "HHHH");
        rec(l, 1275, "HHHH");
        rec(l, 1300, "LLLL");
        assertEquals(1250, l.balance(), EPS);
        assertEquals(1.0, l.bestHitRate(), EPS);
        // -1 at 1200 -> 0 at 1225: -0.5 crosses at 1212.5; +0.5 at 1287.5.
        assertEquals(75, l.transitionWidth(), EPS);
    }

    @Test
    public void lowHitRateEverywhereIsVisible() {
        // The "it's the balls, not the RPM" case: misses both ways at every rung.
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1225, "SSLS");
        rec(l, 1250, "SLHS");
        rec(l, 1275, "LSLL");
        assertTrue(l.bestHitRate() <= 0.25 + EPS);
        assertTrue(l.isBounded());
        assertTrue("wide transition", l.transitionWidth() > 25);
    }

    // ---- next(): where it goes ------------------------------------------------

    @Test
    public void staysUntilTheRungIsFull() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1250, "SSS");
        assertEquals(1250, l.next(1250), EPS);
        rec(l, 1250, "S");
        assertEquals(1275, l.next(1250), EPS);
    }

    @Test
    public void climbsThenFillsTheGap() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1200, "SSSS");
        assertEquals(1225, l.next(1200), EPS);
        rec(l, 1225, "SSSH");
        assertEquals(1250, l.next(1225), EPS);
        rec(l, 1250, "LLLL");   // jumped straight past the band
        assertTrue(l.isBounded());
        assertTrue("nothing left between 1225 and 1250", l.isDone());
    }

    @Test
    public void exploresOutwardFromAllHits() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1250, "HHHH");
        double next = l.next(1250);
        assertTrue(next == 1225 || next == 1275);
    }

    @Test
    public void aimLadderDoesNotWanderOffACentredRung() {
        Ladder aim = new Ladder(1.0, 4, false);
        rec(aim, 0, "HHHH");
        assertEquals(0, aim.next(0), EPS);
        rec(aim, 0, "<<<<");   // now 8 balls, mostly... still half: not mostly high
        assertEquals(0, aim.next(0), EPS);
        rec(aim, 0, "<<");     // 10 balls, 6 left -> mostly high (went left) -> trim down
        assertEquals(-1, aim.next(0), EPS);
    }

    @Test
    public void aNudgedValueRejoinsItsRung() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1250, "SSSS");
        rec(l, 1274, "H");      // someone nudged to 1274 instead of 1275
        assertEquals(1274, l.next(1250), EPS);   // finish that rung, don't start 1275 beside it
    }

    @Test
    public void contradictoryRungsAreNoisyAndStop() {
        Ladder l = new Ladder(25, 4, true);
        rec(l, 1250, "LLLH");
        rec(l, 1275, "SSSH");
        assertTrue(l.isNoisy());
        assertTrue(l.isDone());
    }

    @Test
    public void emptyLadderHasNoAnswer() {
        Ladder l = new Ladder(25, 4, true);
        assertTrue(Double.isNaN(l.balance()));
        assertEquals(1250, l.next(1250), EPS);
    }

    private static void rec(Ladder l, double v, String outcomes) {
        for (char c : outcomes.toCharArray()) {
            Ladder.Result r = c == 'S' || c == '>' ? Ladder.Result.LOW
                    : c == 'L' || c == '<' ? Ladder.Result.HIGH : Ladder.Result.HIT;
            l.record(v, r, c);
        }
    }

    // ---- Outcome mapping -------------------------------------------------------

    @Test
    public void outcomesFeedTheRightLadder() {
        assertEquals(Ladder.Result.LOW, Outcome.SHORT.forRpm());
        assertEquals(Ladder.Result.HIGH, Outcome.LONG.forRpm());
        assertNull("a sideways miss says nothing about RPM", Outcome.LEFT.forRpm());
        // Trim + = turret further left, so a ball that went LEFT had too much.
        assertEquals(Ladder.Result.HIGH, Outcome.LEFT.forAim());
        assertEquals(Ladder.Result.LOW, Outcome.RIGHT.forAim());
        assertEquals("short but on line counts as centred", Ladder.Result.HIT, Outcome.SHORT.forAim());
    }

    // ---- fit -------------------------------------------------------------------

    @Test
    public void fitRecoversTheFlywheelOffsetTerms() {
        // rpm = 900 + 6*d + 20*cos(t) - 35*sin(t), at 3 distances x 5 angles, no noise.
        double[] ds = {40, 60, 80};
        double[] ts = {-90, -45, 0, 45, 90};
        double[][] rows = new double[ds.length * ts.length][];
        double[] y = new double[rows.length];
        int i = 0;
        for (double d : ds) {
            for (double t : ts) {
                rows[i] = ShotMap.rpmRow(d, t);
                double r = Math.toRadians(t);
                y[i++] = 900 + 6 * d + 20 * Math.cos(r) - 35 * Math.sin(r);
            }
        }
        Fit f = ShotMap.fit(rows, y, null);
        assertNotNull(f);
        assertEquals(900, f.coef[0], 1e-6);
        assertEquals(6, f.coef[1], 1e-6);
        assertEquals(20, f.coef[2], 1e-6);
        assertEquals(-35, f.coef[3], 1e-6);
        assertEquals(0, f.sigma[3], 1e-6);
    }

    @Test
    public void fitErrorBarsCoverTheTruthWithNoisySpots() {
        // Same model, 10 RPM of noise on each spot. The true sin term should
        // land inside 3 sigma nearly every time, and the sigma should be sane.
        int covered = 0, runs = 100;
        for (int seed = 0; seed < runs; seed++) {
            Random rnd = new Random(seed);
            double[] ts = {-90, -45, 0, 45, 90, -90, 0, 90};
            double[] ds = {50, 50, 50, 50, 50, 70, 70, 70};
            double[][] rows = new double[ts.length][];
            double[] y = new double[ts.length];
            for (int i = 0; i < ts.length; i++) {
                rows[i] = ShotMap.rpmRow(ds[i], ts[i]);
                double r = Math.toRadians(ts[i]);
                y[i] = 900 + 6 * ds[i] + 20 * Math.cos(r) - 35 * Math.sin(r) + rnd.nextGaussian() * 10;
            }
            Fit f = ShotMap.fit(rows, y, null);
            assertNotNull(f);
            assertTrue(f.sigma[3] > 0 && f.sigma[3] < 20);
            if (Math.abs(f.coef[3] - (-35)) <= 3 * f.sigma[3]) covered++;
        }
        assertTrue("covered " + covered + "/" + runs, covered >= 0.9 * runs);
    }

    @Test
    public void weightsFavourTheSureSpots() {
        // Two spots disagree about a constant; the one weighted 100x wins.
        double[][] rows = {{1}, {1}};
        Fit f = ShotMap.fit(rows, new double[]{1000, 1100}, new double[]{100, 1});
        assertNotNull(f);
        assertEquals(1000 + 100.0 / 101, f.coef[0], 1e-6);
    }

    @Test
    public void fitRefusesWhenEverySpotIsAtOneTurretAngle() {
        double[][] rows = {ShotMap.rpmRow(40, 0), ShotMap.rpmRow(60, 0),
                ShotMap.rpmRow(80, 0), ShotMap.rpmRow(100, 0)};
        assertNull(ShotMap.fit(rows, new double[]{1100, 1200, 1300, 1400}, null));
    }

    @Test
    public void fitRefusesTooFewRows() {
        assertNull(ShotMap.fit(new double[][]{ShotMap.aimRow(0), ShotMap.aimRow(90)},
                new double[]{1, 2}, null));
    }

    @Test
    public void distinctAnglesGroupsNearbyOnes() {
        assertEquals(3, ShotMap.distinctAngles(new double[]{-1, 2, 44, 46, 90}, 10));
    }

    // ---- CSV ---------------------------------------------------------------------

    @Test
    public void csvRowsMatchTheirHeaders() {
        String shot = ShotMap.shotRow(1.5, "6494", 2, 7, "red, audience wall", 33, 9, 0, 61.2,
                23.4, 1250, 1245, 1251, 1, 85, 0.5, 67, true, Outcome.SHORT);
        assertEquals(ShotMap.SHOT_HEADER.split(",").length, shot.split(",", -1).length);
        assertTrue("commas in names must not split columns", shot.contains("red; audience wall"));

        Ladder rpm = new Ladder(25, 4, true);
        rec(rpm, 1250, "HHSL");
        String spot = ShotMap.spotRow("6494", 2, 61.2, 23.4, 4, rpm, new Ladder(1, 4, false));
        assertEquals(ShotMap.SPOT_HEADER.split(",").length, spot.split(",", -1).length);
    }
}
