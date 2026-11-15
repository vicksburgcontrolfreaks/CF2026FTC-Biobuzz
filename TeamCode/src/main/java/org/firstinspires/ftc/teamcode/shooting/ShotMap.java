package org.firstinspires.ftc.teamcode.shooting;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/*
 * ShotMap: the bookkeeping behind Shot Map Test. Plain numbers, no hardware,
 * tested on a laptop (ShotMapTest).
 *
 * BALLS ARE RANDOM, SO EVERY RPM IS A PROBABILITY (Shane, 2026-11-13)
 * At ~1250 RPM the ball-to-ball scatter (ball condition, feed, wheel recovery)
 * is likely bigger than one 25 RPM step. So one ball at an RPM proves nothing:
 * a short ball at 1260 between hits at 1240 and 1280 is just a bad ball. An
 * earlier version kept only the extremes and was wrecked by exactly that.
 *
 * So: a LADDER. Rungs every RUNG RPM, several balls on each, and a tally of
 * short / hit / long per rung:
 *
 *     1300   L L H L    hit 25%   score +0.75
 *     1275   H L H H    hit 75%   score +0.25
 *     1250   H H S H    hit 75%   score -0.25
 *     1225   S H S S    hit 25%   score -0.75
 *
 * score = (longs - shorts) / balls, from -1 (all short) to +1 (all long).
 *
 * THE ANSWER is the BALANCE POINT, where score crosses 0 -- longs and shorts
 * cancel. It is interpolated between rungs and uses EVERY ball, misses
 * included, so a few lucky hits can't drag it. Two more numbers come free:
 *   - best hit rate: if the best rung only hits 60%, the limit is the balls and
 *     the mechanism, not the RPM, and a finer table buys nothing;
 *   - transition width: RPM from score -0.5 to +0.5 -- how spread out the
 *     misses are, i.e. how much RPM (or distance) error the shot forgives.
 *
 * The same Ladder does the aim trim, with LEFT/RIGHT as low/high.
 *
 * THE FLYWHEEL OFFSET
 * On 6494 the flywheels sit ~0.5-1 in left of the turret's centre. The ball
 * comes up at a fixed spot on the ROBOT and the deflector turns with the turret,
 * so its effect depends only on the TURRET ANGLE: sideways at 0, fore/aft at
 * +-90. That is a cos/sin in turret angle -- fit() finds a handful of numbers,
 * WITH a +- on each so it is clear whether the effect beats the noise. On 5618
 * the turret is centred over the flywheels and the terms should come out
 * inside their +-.
 */
public final class ShotMap {

    private ShotMap() {}

    /** What the operator saw. One press per ball. */
    public enum Outcome {
        HIT, SHORT, LONG, LEFT, RIGHT;

        /** What this ball says about the RPM, or null if nothing (it went sideways). */
        public Ladder.Result forRpm() {
            switch (this) {
                case HIT:   return Ladder.Result.HIT;
                case SHORT: return Ladder.Result.LOW;
                case LONG:  return Ladder.Result.HIGH;
                default:    return null;
            }
        }

        /**
         * What this ball says about the AIM. Trim + = turret further
         * counter-clockwise (left), so a ball that went LEFT had too much. A
         * short or long ball was on line, so it counts as centred for the aim.
         */
        public Ladder.Result forAim() {
            switch (this) {
                case LEFT:  return Ladder.Result.HIGH;
                case RIGHT: return Ladder.Result.LOW;
                default:    return Ladder.Result.HIT;
            }
        }

        public char letter() {
            switch (this) {
                case HIT:   return 'H';
                case SHORT: return 'S';
                case LONG:  return 'L';
                case LEFT:  return '<';
                default:    return '>';
            }
        }
    }

    // ==========================================================================
    // Ladder
    // ==========================================================================

    public static final class Ladder {

        public enum Result { LOW, HIT, HIGH }

        /** One rung: every ball shot at this value, in order. */
        public static final class Rung {
            public final double value;
            private int lows, hits, highs;
            private final StringBuilder seq = new StringBuilder();

            Rung(double value) { this.value = value; }

            public int n()     { return lows + hits + highs; }
            public int lows()  { return lows; }
            public int hits()  { return hits; }
            public int highs() { return highs; }
            /** (highs - lows) / n: -1 all low, +1 all high. */
            public double score() { return n() == 0 ? 0 : (double) (highs - lows) / n(); }
            public double hitRate() { return n() == 0 ? 0 : (double) hits / n(); }
            public boolean mostlyLow()  { return lows * 2 > n(); }
            public boolean mostlyHigh() { return highs * 2 > n(); }
            public String sequence() { return seq.toString(); }
        }

        private final TreeMap<Double, Rung> rungs = new TreeMap<>();
        private final double spacing;
        private final int ballsPerRung;
        private final boolean explore;

        /**
         * @param spacing      rung spacing (RPM, or degrees of trim)
         * @param ballsPerRung balls before a rung counts as measured
         * @param explore      true = step outward to find BOTH sides even when the
         *                     current rung is mostly hits (the RPM search). False =
         *                     only move when the rung is mostly low or high (the aim:
         *                     we want its centre, and deliberately missing sideways
         *                     wastes balls)
         */
        public Ladder(double spacing, int ballsPerRung, boolean explore) {
            this.spacing = spacing;
            this.ballsPerRung = Math.max(1, ballsPerRung);
            this.explore = explore;
        }

        /** Values are rounded to 0.01 so 1250 and 1250.0000001 are one rung. */
        public void record(double value, Result result, char letter) {
            double key = Math.round(value * 100) / 100.0;
            Rung r = rungs.get(key);
            if (r == null) {
                r = new Rung(key);
                rungs.put(key, r);
            }
            switch (result) {
                case LOW:  r.lows++;  break;
                case HIT:  r.hits++;  break;
                case HIGH: r.highs++; break;
            }
            r.seq.append(letter);
        }

        public boolean isEmpty() { return rungs.isEmpty(); }

        public int balls() {
            int n = 0;
            for (Rung r : rungs.values()) n += r.n();
            return n;
        }

        /** Rungs, LOWEST value first. */
        public List<Rung> rungs() { return new ArrayList<>(rungs.values()); }

        public Rung rungAt(double value) {
            return rungs.get(Math.round(value * 100) / 100.0);
        }

        private boolean full(Rung r) { return r.n() >= ballsPerRung; }

        /** Highest full rung that is mostly LOW, or null. */
        public Rung highestLow() {
            Rung best = null;
            for (Rung r : rungs.values()) if (full(r) && r.mostlyLow()) best = r;
            return best;
        }

        /** Lowest full rung that is mostly HIGH, or null. */
        public Rung lowestHigh() {
            for (Rung r : rungs.values()) if (full(r) && r.mostlyHigh()) return r;
            return null;
        }

        /** A mostly-low rung below a mostly-high one: the balance point is pinned between them. */
        public boolean isBounded() {
            Rung lo = highestLow(), hi = lowestHigh();
            return lo != null && hi != null && lo.value < hi.value;
        }

        /** A mostly-low rung at or ABOVE a mostly-high one: the shot isn't repeatable here. */
        public boolean isNoisy() {
            Rung lo = highestLow(), hi = lowestHigh();
            return lo != null && hi != null && lo.value >= hi.value;
        }

        /**
         * Where score crosses `level`, interpolated between rungs. If it crosses
         * more than once (noise, or a plateau of all-hit rungs), the middle of the
         * first and last crossing. NaN if it never does.
         */
        public double crossing(double level) {
            List<Rung> rs = rungs();
            double first = Double.NaN, last = Double.NaN;
            for (int i = 0; i + 1 < rs.size(); i++) {
                Rung a = rs.get(i), b = rs.get(i + 1);
                double sa = a.score() - level, sb = b.score() - level;
                if (sa == 0 && sb == 0) continue;       // flat on the level: its ends are counted
                if (sa <= 0 && sb >= 0) {
                    double x = a.value + (-sa) / (sb - sa) * (b.value - a.value);
                    if (Double.isNaN(first)) first = x;
                    last = x;
                }
            }
            // A single rung sitting exactly on the level with nothing either side.
            if (Double.isNaN(first) && rs.size() == 1 && rs.get(0).score() == level) {
                return rs.get(0).value;
            }
            return Double.isNaN(first) ? Double.NaN : (first + last) / 2;
        }

        /** The answer: where longs and shorts cancel. NaN until it crosses. */
        public double balance() { return crossing(0); }

        /**
         * balance() when there is one; otherwise a rough guess from whatever is
         * there -- each rung nudged by its score toward the side it is missing
         * (all-high = about a rung too far up), averaged by balls. For the AIM,
         * which only moves when a rung is mostly sideways and so often never
         * crosses. NaN only when empty.
         */
        public double estimate() {
            double b = balance();
            if (!Double.isNaN(b)) return b;
            double sum = 0;
            int n = 0;
            for (Rung r : rungs.values()) {
                sum += (r.value - r.score() * spacing) * r.n();
                n += r.n();
            }
            return n == 0 ? Double.NaN : sum / n;
        }

        /** RPM from score -0.5 to +0.5: how spread the misses are. NaN until both cross. */
        public double transitionWidth() {
            double lo = crossing(-0.5), hi = crossing(0.5);
            return Double.isNaN(lo) || Double.isNaN(hi) ? Double.NaN : hi - lo;
        }

        /** Best hit rate on any FULL rung (0 if none). */
        public double bestHitRate() {
            double best = 0;
            for (Rung r : rungs.values()) if (full(r)) best = Math.max(best, r.hitRate());
            return best;
        }

        /**
         * Rough 1-sigma uncertainty of balance(): the transition width over
         * 2 * sqrt(balls in the transition). Used to WEIGHT spots in fit(), not as
         * a statistic to quote. Falls back to the rung spacing when there is no
         * width yet.
         */
        public double balanceSigma() {
            Rung lo = highestLow(), hi = lowestHigh();
            int n = 0;
            for (Rung r : rungs.values()) {
                if ((lo == null || r.value >= lo.value) && (hi == null || r.value <= hi.value)) n += r.n();
            }
            double w = transitionWidth();
            if (Double.isNaN(w) || w <= 0) w = spacing;
            return w / (2 * Math.sqrt(Math.max(1, n)));
        }

        /** Nothing left to shoot: bounded, and every rung between the two edges is full. */
        public boolean isDone() {
            if (isNoisy()) return true;
            return isBounded() && gapToFill() == null;
        }

        /** The first value between the edges still needing balls, or null. */
        private Double gapToFill() {
            Rung lo = highestLow(), hi = lowestHigh();
            if (lo == null || hi == null) return null;
            double prev = lo.value;
            for (Rung r : rungs.subMap(lo.value, false, hi.value, false).values()) {
                if (!full(r)) return r.value;                       // finish a partial rung
                if (r.value - prev > 1.5 * spacing) return mid(prev, r.value);
                prev = r.value;
            }
            if (hi.value - prev > 1.5 * spacing) return mid(prev, hi.value);
            return null;
        }

        private double mid(double a, double b) {
            // Keep to the rung grid measured from `a`, so rungs line up.
            return a + Math.max(1, Math.round((b - a) / 2 / spacing)) * spacing;
        }

        /** Snap to an existing rung within half a spacing, so a nudged value rejoins it. */
        private double snap(double v) {
            Map.Entry<Double, Rung> e = rungs.floorEntry(v + spacing / 2);
            if (e != null && Math.abs(e.getKey() - v) < spacing / 2) return e.getKey();
            return v;
        }

        /**
         * The value to shoot next. Stays on `current` until its rung is full;
         * then climbs if it was mostly low, drops if mostly high, fills in between
         * once both sides are found, and (explore only) steps outward from a rung
         * of mostly hits toward whichever side looks nearer.
         */
        public double next(double current) {
            Rung here = rungAt(current);
            if (here == null || !full(here)) return current;
            if (isNoisy()) return current;

            Double fill = gapToFill();
            if (fill != null) return snap(fill);
            if (isBounded()) return current;   // done

            Rung lo = highestLow(), hi = lowestHigh();
            double minFull = Double.NaN, maxFull = Double.NaN;
            for (Rung r : rungs.values()) {
                if (!full(r)) continue;
                if (Double.isNaN(minFull)) minFull = r.value;
                maxFull = r.value;
            }
            if (lo != null) return snap(maxFull + spacing);   // need the high side: go above everything measured
            if (hi != null) return snap(minFull - spacing);   // need the low side

            // Every full rung is mostly hits (or mixed). Only the RPM search explores.
            if (!explore) {
                if (here.mostlyLow()) return snap(current + spacing);
                if (here.mostlyHigh()) return snap(current - spacing);
                return current;
            }
            double sUp = rungAt(maxFull).score(), sDown = rungAt(minFull).score();
            double toHigh = 0.5 - sUp, toLow = 0.5 + sDown;   // how far each end is from "mostly"
            return toHigh <= toLow ? snap(maxFull + spacing) : snap(minFull - spacing);
        }
    }

    // ==========================================================================
    // fit: weighted least squares for  value = c0 + c1*x1 + ...
    // ==========================================================================

    /** Coefficients and a 1-sigma +- on each. */
    public static final class Fit {
        public final double[] coef;
        public final double[] sigma;
        /** Weighted RMS of the residuals, in the value's units; NaN with no spare rows. */
        public final double rms;

        Fit(double[] coef, double[] sigma, double rms) {
            this.coef = coef;
            this.sigma = sigma;
            this.rms = rms;
        }
    }

    /**
     * Weighted least squares. rows[i] are one observation's regressors (put 1.0
     * first for a constant); values[i] what was measured; weights[i] its weight
     * (1/sigma^2), or null for all equal. Returns null with too few rows or when
     * the regressors can't be told apart (every spot at one turret angle, say).
     *
     * The +- uses the scatter of the residuals (not the weights' absolute size),
     * so it is honest even when the per-spot sigmas are only rough. With exactly
     * as many rows as terms there is no scatter to measure and the +- is NaN.
     */
    public static Fit fit(double[][] rows, double[] values, double[] weights) {
        int n = rows.length;
        if (n == 0 || n != values.length || (weights != null && weights.length != n)) return null;
        int k = rows[0].length;
        if (n < k) return null;

        double[][] ata = new double[k][k];
        double[] aty = new double[k];
        for (int r = 0; r < n; r++) {
            double w = weights == null ? 1 : weights[r];
            for (int i = 0; i < k; i++) {
                for (int j = 0; j < k; j++) ata[i][j] += w * rows[r][i] * rows[r][j];
                aty[i] += w * rows[r][i] * values[r];
            }
        }
        double[][] inv = invert(ata);
        if (inv == null) return null;

        double[] c = new double[k];
        for (int i = 0; i < k; i++) for (int j = 0; j < k; j++) c[i] += inv[i][j] * aty[j];

        double ss = 0, wsum = 0;
        for (int r = 0; r < n; r++) {
            double w = weights == null ? 1 : weights[r];
            double pred = 0;
            for (int i = 0; i < k; i++) pred += c[i] * rows[r][i];
            ss += w * (values[r] - pred) * (values[r] - pred);
            wsum += w;
        }
        double[] sigma = new double[k];
        double rms = Double.NaN;
        if (n > k) {
            double s2 = ss / (n - k);
            for (int i = 0; i < k; i++) sigma[i] = Math.sqrt(Math.max(0, s2 * inv[i][i]));
            rms = Math.sqrt(ss / wsum);
        } else {
            java.util.Arrays.fill(sigma, Double.NaN);
        }
        return new Fit(c, sigma, rms);
    }

    /** Gauss-Jordan with partial pivoting. Null if (nearly) singular. */
    private static double[][] invert(double[][] a) {
        int k = a.length;
        double[][] m = new double[k][2 * k];
        double scale = 0;
        for (int i = 0; i < k; i++) {
            for (int j = 0; j < k; j++) {
                m[i][j] = a[i][j];
                scale = Math.max(scale, Math.abs(a[i][j]));
            }
            m[i][k + i] = 1;
        }
        for (int col = 0; col < k; col++) {
            int pivot = col;
            for (int r = col + 1; r < k; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
            }
            if (Math.abs(m[pivot][col]) < 1e-9 * Math.max(1, scale)) return null;
            double[] tmp = m[col]; m[col] = m[pivot]; m[pivot] = tmp;
            double p = m[col][col];
            for (int j = 0; j < 2 * k; j++) m[col][j] /= p;
            for (int r = 0; r < k; r++) {
                if (r == col) continue;
                double f = m[r][col];
                if (f == 0) continue;
                for (int j = 0; j < 2 * k; j++) m[r][j] -= f * m[col][j];
            }
        }
        double[][] out = new double[k][k];
        for (int i = 0; i < k; i++) System.arraycopy(m[i], k, out[i], 0, k);
        return out;
    }

    /** Regressors for the RPM model:  rpm = c0 + c1*distance + a*cos(turret) + b*sin(turret). */
    public static double[] rpmRow(double distanceIn, double turretDeg) {
        double t = Math.toRadians(turretDeg);
        return new double[]{1, distanceIn, Math.cos(t), Math.sin(t)};
    }

    /** Regressors for the aim model:  trim = c + a*cos(turret) + b*sin(turret). */
    public static double[] aimRow(double turretDeg) {
        double t = Math.toRadians(turretDeg);
        return new double[]{1, Math.cos(t), Math.sin(t)};
    }

    /**
     * How many distinct turret angles a set of spots covers: angles within
     * `binDeg` of the start of a group count as that group. (Not rounding to
     * fixed bins -- 44 and 46 would land in different ones.)
     */
    public static int distinctAngles(double[] turretDeg, double binDeg) {
        if (turretDeg.length == 0) return 0;
        double[] t = turretDeg.clone();
        java.util.Arrays.sort(t);
        int groups = 1;
        double groupStart = t[0];
        for (double v : t) {
            if (v - groupStart > binDeg) {
                groups++;
                groupStart = v;
            }
        }
        return groups;
    }

    // ==========================================================================
    // CSV
    // ==========================================================================

    public static final String SHOT_HEADER =
            "kind,time_s,robot,spot,shot,start,x,y,heading_deg,distance_in,turret_deg,"
                    + "target_rpm,left_rpm,right_rpm,in_burst,dip_rpm,aim_trim_deg,"
                    + "deflector_deg,pose_trusted,outcome";

    /** One ball. Units: inches, degrees, RPM, seconds. NaN prints as empty. */
    public static String shotRow(double timeSec, String robot, int spot, int shot, String start,
                                 double x, double y, double headingDeg, double distance,
                                 double turretDeg, double targetRpm, double leftRpm,
                                 double rightRpm, int inBurst, double dipRpm, double aimTrimDeg,
                                 double deflectorDeg, boolean poseTrusted, Outcome outcome) {
        return "shot," + num(timeSec, 2) + "," + csv(robot) + "," + spot + "," + shot + ","
                + csv(start) + "," + num(x, 1) + "," + num(y, 1) + "," + num(headingDeg, 1) + ","
                + num(distance, 1) + "," + num(turretDeg, 1) + "," + num(targetRpm, 0) + ","
                + num(leftRpm, 0) + "," + num(rightRpm, 0) + "," + inBurst + ","
                + num(dipRpm, 0) + "," + num(aimTrimDeg, 2) + "," + num(deflectorDeg, 1) + ","
                + poseTrusted + "," + outcome;
    }

    public static final String SPOT_HEADER =
            "kind,robot,spot,distance_in,turret_deg,balls,rpm_balance,rpm_sigma,rpm_width,"
                    + "rpm_best_hit_rate,rpm_bounded,rpm_noisy,aim_balance,aim_width,aim_noisy";

    /** One finished spot: where it was and what the two ladders found there. */
    public static String spotRow(String robot, int spot, double distance, double turretDeg,
                                 int balls, Ladder rpm, Ladder aim) {
        return "spot," + csv(robot) + "," + spot + "," + num(distance, 1) + ","
                + num(turretDeg, 1) + "," + balls + ","
                + num(rpm.balance(), 0) + "," + num(rpm.balanceSigma(), 1) + ","
                + num(rpm.transitionWidth(), 0) + "," + num(rpm.bestHitRate(), 2) + ","
                + rpm.isBounded() + "," + rpm.isNoisy() + ","
                + num(aim.balance(), 2) + "," + num(aim.transitionWidth(), 2) + "," + aim.isNoisy();
    }

    private static String num(double v, int decimals) {
        return Double.isNaN(v) ? "" : String.format(Locale.US, "%." + decimals + "f", v);
    }

    private static String csv(String s) {
        return s == null ? "" : s.replace(",", ";");
    }
}
