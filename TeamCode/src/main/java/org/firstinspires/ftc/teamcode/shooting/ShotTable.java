package org.firstinspires.ftc.teamcode.shooting;

import java.util.Arrays;

/*
 * ShotTable: distance to the hive -> flywheel speed (Phase 4 of the turret plan).
 *
 * We don't calculate the shot from physics. Too much we can't measure well
 * (ball spin, compression, drag). Instead: shoot from a few distances, write down
 * the RPM that scores at each one, and draw straight lines between them.
 *
 *   distance (in)   36     60     84     108
 *   RPM           2100   2400   2800   3300      <- made-up example
 *
 * At 72 in. we're halfway between 60 and 84, so 2600.
 *
 * The distance is OdometryAim.Solution.distance: FLAT distance from the turret
 * axis to the opening. Height is left out on purpose -- the opening is always
 * the same height, so distance alone decides the shot.
 *
 * OUTSIDE THE TABLE WE DON'T GUESS. Past the last row, the value stays at the
 * last row and inRange() says false. Shot speed isn't a straight line in
 * distance, so extending the line past real data is how you miss in a match.
 * Want a longer shot? Go measure that distance and add a row.
 *
 * Plain numbers, no hardware: tested on a laptop (ShotTableTest).
 */
public final class ShotTable {

    // ---- OUR robot's table --------------------------------------------------
    // TODO(measure, Phase 4): EMPTY until we shoot. Rows sorted by distance, nearest
    // first. Units: inches, and flywheel RPM (the same RPM ShootAprilTag uses).
    // Rows every ~12-24 in. across where we actually shoot; more rows where the
    // RPM changes fastest.
    public static double[] DISTANCES = {};
    public static double[] RPMS = {};

    /** Our robot's distance -> RPM table, from the arrays above. */
    public static ShotTable forRobot() {
        return new ShotTable(DISTANCES, RPMS);
    }

    private final double[] distances;
    private final double[] values;

    /**
     * @param distances inches, strictly increasing
     * @param values    one per distance (RPM, or a hood angle -- anything that
     *                  changes smoothly with distance)
     */
    public ShotTable(double[] distances, double[] values) {
        if (distances.length != values.length) {
            throw new IllegalArgumentException("ShotTable: " + distances.length
                    + " distances but " + values.length + " values");
        }
        for (int i = 0; i < distances.length; i++) {
            if (Double.isNaN(distances[i]) || Double.isNaN(values[i])) {
                throw new IllegalArgumentException("ShotTable: row " + i + " has a NaN");
            }
            if (i > 0 && distances[i] <= distances[i - 1]) {
                throw new IllegalArgumentException("ShotTable: distances must go up, row " + i
                        + " (" + distances[i] + ") is not past row " + (i - 1)
                        + " (" + distances[i - 1] + ")");
            }
        }
        this.distances = distances.clone();
        this.values = values.clone();
    }

    /** False until there's at least one row. Don't spin up from an empty table. */
    public boolean isConfigured() {
        return distances.length > 0;
    }

    /** True if this distance is between the first and last rows (inclusive). */
    public boolean inRange(double distance) {
        return isConfigured()
                && distance >= distances[0]
                && distance <= distances[distances.length - 1];
    }

    /**
     * The value for this distance: a straight line between the two rows around
     * it, or the end row's value if it's off either end. NaN if the table is empty.
     */
    public double lookup(double distance) {
        int n = distances.length;
        if (n == 0) return Double.NaN;
        if (distance <= distances[0]) return values[0];
        if (distance >= distances[n - 1]) return values[n - 1];

        // First row at or past this distance. binarySearch returns -(insertion point) - 1 on a miss.
        int hi = Arrays.binarySearch(distances, distance);
        if (hi >= 0) return values[hi];
        hi = -hi - 1;
        int lo = hi - 1;

        double t = (distance - distances[lo]) / (distances[hi] - distances[lo]);
        return values[lo] + t * (values[hi] - values[lo]);
    }

    /** Rows in the table. */
    public int size() {
        return distances.length;
    }
}
