package org.firstinspires.ftc.teamcode.shooting;

import java.util.Locale;

/*
 * ShotMap: the bookkeeping behind Shot Map Test. Plain numbers, no hardware,
 * tested on a laptop (ShotMapTest).
 *
 * THE IDEA: FIND THE EDGES, NOT ONE GOOD SHOT
 * "2300 scored" tells you almost nothing -- 2200 and 2400 might have scored too,
 * or 2300 might be one lucky ball on the edge of missing. So at each spot we hunt
 * for the two EDGES of the hit band: the RPM where shots stop landing short, and
 * the RPM where they start going long. The middle of the band is what goes in
 * ShotTable. The WIDTH of the band is the tolerance -- how much RPM (or
 * localization) error the shot forgives -- which is the number that says whether
 * any of this is worth more precision.
 *
 * Bracket does the hunting. Every outcome narrows one of the two gaps:
 *
 *     too low ... maxLow  [gap]  minHit ... maxHit  [gap]  minHigh ... too high
 *
 * suggest() probes the middle of the wider gap until both are narrower than the
 * resolution. Before there is a hit it steps toward one; once there is a hit with
 * nothing past it, it steps outward to find the far side.
 *
 * The same Bracket works for the aim trim, with LEFT/RIGHT in place of
 * SHORT/LONG.
 *
 * THE FLYWHEEL OFFSET (Shane, 2026-11-13)
 * On 6494 the flywheels sit ~0.5-1 in left of the turret's centre. The ball
 * comes up at a fixed spot on the ROBOT and the deflector turns with the turret,
 * so where the ball lands on the deflector -- and so its effect -- depends only on
 * the TURRET ANGLE: sideways at 0 (a left/right aim bias), fore/aft at +-90 (long
 * one way, short the other). That is a sine/cosine in turret angle, so fit() finds
 * a handful of numbers, not a grid. On 5618 the turret is centred over the
 * flywheels and the fit should come out near zero -- which is worth confirming,
 * not assuming.
 */
public final class ShotMap {

    private ShotMap() {}

    /** What the operator saw. One press per ball. */
    public enum Outcome {
        HIT, SHORT, LONG, LEFT, RIGHT;

        /** What this means for the RPM search, or null if it says nothing about RPM. */
        public Bracket.Result forRpm() {
            switch (this) {
                case HIT:   return Bracket.Result.HIT;
                case SHORT: return Bracket.Result.LOW;
                case LONG:  return Bracket.Result.HIGH;
                default:    return null;
            }
        }

        /**
         * What this means for the AIM-TRIM search. Trim is + = turret further
         * counter-clockwise (left), so a ball that went LEFT had too much.
         */
        public Bracket.Result forAim() {
            switch (this) {
                case HIT:   return Bracket.Result.HIT;
                case LEFT:  return Bracket.Result.HIGH;
                case RIGHT: return Bracket.Result.LOW;
                default:    return null;
            }
        }
    }

    // ==========================================================================
    // Bracket: hunt for the edges of a "hit" band in one variable.
    // ==========================================================================

    public static final class Bracket {

        public enum Result { LOW, HIT, HIGH }

        private double maxLow = Double.NaN;    // highest value that was too low
        private double minHigh = Double.NaN;   // lowest value that was too high
        private double minHit = Double.NaN;
        private double maxHit = Double.NaN;
        private int lows, hits, highs;

        public void record(double value, Result result) {
            switch (result) {
                case LOW:
                    maxLow = Double.isNaN(maxLow) ? value : Math.max(maxLow, value);
                    lows++;
                    break;
                case HIGH:
                    minHigh = Double.isNaN(minHigh) ? value : Math.min(minHigh, value);
                    highs++;
                    break;
                case HIT:
                    minHit = Double.isNaN(minHit) ? value : Math.min(minHit, value);
                    maxHit = Double.isNaN(maxHit) ? value : Math.max(maxHit, value);
                    hits++;
                    break;
            }
        }

        public int lows()  { return lows; }
        public int hits()  { return hits; }
        public int highs() { return highs; }
        public boolean isEmpty() { return lows + hits + highs == 0; }
        public double maxLow()  { return maxLow; }
        public double minHigh() { return minHigh; }
        public double minHit()  { return minHit; }
        public double maxHit()  { return maxHit; }

        /**
         * True when the record contradicts itself: something too LOW at or above
         * a hit, or too HIGH at or below one. That is the shot not being
         * repeatable at this value -- worth knowing, and a reason to take more
         * shots before trusting the band.
         */
        public boolean isNoisy() {
            return (!Double.isNaN(maxLow) && !Double.isNaN(minHit) && maxLow >= minHit)
                    || (!Double.isNaN(minHigh) && !Double.isNaN(maxHit) && minHigh <= maxHit)
                    || (!Double.isNaN(maxLow) && !Double.isNaN(minHigh) && maxLow >= minHigh);
        }

        /** Lower edge of the band, halfway across the gap; NaN until both sides are seen. */
        public double lowEdge() {
            return Double.isNaN(maxLow) || Double.isNaN(minHit) ? Double.NaN : (maxLow + minHit) / 2;
        }

        /** Upper edge of the band, halfway across the gap; NaN until both sides are seen. */
        public double highEdge() {
            return Double.isNaN(minHigh) || Double.isNaN(maxHit) ? Double.NaN : (maxHit + minHigh) / 2;
        }

        /** Both edges found: the band's middle and width are real, not provisional. */
        public boolean isBounded() {
            return !Double.isNaN(lowEdge()) && !Double.isNaN(highEdge());
        }

        /**
         * Best estimate of the middle of the band. With both edges, the middle of
         * them; with hits only, the middle of the hits (provisional); with no hit
         * yet, NaN.
         */
        public double centre() {
            if (isBounded()) return (lowEdge() + highEdge()) / 2;
            if (hits > 0) return (minHit + maxHit) / 2;
            return Double.NaN;
        }

        /** Band width between the edges, or NaN until bounded. */
        public double width() {
            return isBounded() ? highEdge() - lowEdge() : Double.NaN;
        }

        /** True once both gaps are narrower than the resolution. Nothing left to probe. */
        public boolean isDone(double resolution) {
            return isBounded() && !isNoisy()
                    && minHit - maxLow <= resolution && minHigh - maxHit <= resolution;
        }

        /**
         * The next value to try.
         *
         * @param current    what is set now; returned when there is nothing to go on
         * @param step       how far to step when exploring outward (no edge seen yet)
         * @param resolution stop splitting a gap narrower than this; answers are rounded to it
         */
        public double suggest(double current, double step, double resolution) {
            if (isEmpty()) return current;

            if (hits == 0) {
                if (!Double.isNaN(maxLow) && !Double.isNaN(minHigh)) {
                    // Short below, long above, nothing in between yet: split it.
                    return round(maxLow < minHigh ? (maxLow + minHigh) / 2 : current, resolution);
                }
                return round(!Double.isNaN(maxLow) ? maxLow + step : minHigh - step, resolution);
            }

            // Have a hit. Work on whichever side is less known: unexplored first,
            // then the wider gap. Exploring outward from the hits finds the far side.
            double lowGap = Double.isNaN(maxLow) ? Double.POSITIVE_INFINITY : minHit - maxLow;
            double highGap = Double.isNaN(minHigh) ? Double.POSITIVE_INFINITY : minHigh - maxHit;
            if (lowGap <= resolution && highGap <= resolution) return round(centre(), resolution);

            if (lowGap >= highGap) {
                return round(Double.isNaN(maxLow) ? minHit - step : (maxLow + minHit) / 2, resolution);
            }
            return round(Double.isNaN(minHigh) ? maxHit + step : (maxHit + minHigh) / 2, resolution);
        }

        /** One line for the Driver Station. */
        public String describe(String unit) {
            if (isEmpty()) return "no shots yet";
            StringBuilder sb = new StringBuilder();
            sb.append(Double.isNaN(maxLow) ? "low ?" : fmt("low<=%.0f", maxLow));
            sb.append(hits == 0 ? " | no hit" : fmt(" | hit %.0f-%.0f (%d)", minHit, maxHit, hits));
            sb.append(Double.isNaN(minHigh) ? " | high ?" : fmt(" | high>=%.0f", minHigh));
            if (isBounded()) sb.append(fmt("  => %.0f +-%.0f %s", centre(), width() / 2, unit));
            if (isNoisy()) sb.append("  NOISY");
            return sb.toString();
        }

        private static double round(double v, double resolution) {
            return resolution > 0 ? Math.round(v / resolution) * resolution : v;
        }
    }

    // ==========================================================================
    // fit: least squares for  value = c0 + c1*x1 + c2*x2 + ...
    // ==========================================================================

    /**
     * Ordinary least squares. rows[i] is one observation's regressors (put a 1.0
     * first for a constant term); values[i] is what was measured. Returns the
     * coefficients, or null if there are too few rows or the regressors don't
     * vary enough to tell the terms apart (all spots at one turret angle, say).
     */
    public static double[] fit(double[][] rows, double[] values) {
        int n = rows.length;
        if (n == 0 || n != values.length) return null;
        int k = rows[0].length;
        if (n < k) return null;

        // Normal equations: (A^T A) c = A^T y
        double[][] m = new double[k][k + 1];
        for (int r = 0; r < n; r++) {
            for (int i = 0; i < k; i++) {
                for (int j = 0; j < k; j++) m[i][j] += rows[r][i] * rows[r][j];
                m[i][k] += rows[r][i] * values[r];
            }
        }
        // Gaussian elimination with partial pivoting.
        for (int col = 0; col < k; col++) {
            int pivot = col;
            for (int r = col + 1; r < k; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
            }
            double scale = 0;
            for (int j = 0; j < k; j++) scale = Math.max(scale, Math.abs(m[col][j]));
            if (Math.abs(m[pivot][col]) < 1e-9 * Math.max(1, scale)) return null;
            double[] tmp = m[col]; m[col] = m[pivot]; m[pivot] = tmp;
            for (int r = 0; r < k; r++) {
                if (r == col) continue;
                double f = m[r][col] / m[col][col];
                for (int j = col; j <= k; j++) m[r][j] -= f * m[col][j];
            }
        }
        double[] c = new double[k];
        for (int i = 0; i < k; i++) c[i] = m[i][k] / m[i][i];
        return c;
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
            "kind,robot,spot,distance_in,turret_deg,shots,rpm_low_edge,rpm_high_edge,rpm_centre,"
                    + "rpm_width,rpm_noisy,aim_low_edge,aim_high_edge,aim_centre,aim_width,aim_noisy";

    /** One finished spot: where it was and the two bands found there. */
    public static String spotRow(String robot, int spot, double distance, double turretDeg,
                                 int shots, Bracket rpm, Bracket aim) {
        return "spot," + csv(robot) + "," + spot + "," + num(distance, 1) + ","
                + num(turretDeg, 1) + "," + shots + ","
                + num(rpm.lowEdge(), 0) + "," + num(rpm.highEdge(), 0) + ","
                + num(rpm.centre(), 0) + "," + num(rpm.width(), 0) + "," + rpm.isNoisy() + ","
                + num(aim.lowEdge(), 2) + "," + num(aim.highEdge(), 2) + ","
                + num(aim.centre(), 2) + "," + num(aim.width(), 2) + "," + aim.isNoisy();
    }

    private static String num(double v, int decimals) {
        return Double.isNaN(v) ? "" : String.format(Locale.US, "%." + decimals + "f", v);
    }

    private static String csv(String s) {
        return s == null ? "" : s.replace(",", ";");
    }

    private static String fmt(String f, Object... args) {
        return String.format(Locale.US, f, args);
    }
}
