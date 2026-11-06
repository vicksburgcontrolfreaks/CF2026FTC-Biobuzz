package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.teamcode.field.HiveGeometry.Tilt;
import org.firstinspires.ftc.teamcode.field.HiveTags;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

/*
 * LaunchReach: "can the turret point at the hive from this launch spot?"
 * Checked at INIT, before the robot drives, like PathSafety.
 *
 * The turret only turns -140..+140 (Turret.MIN_ANGLE / MAX_ANGLE). If a route
 * launches from a spot where the hive is behind the turret, it parks at a
 * limit and misses -- and nobody finds out until the match.
 *
 * It checks BOTH hive tilts, because we don't know which cell will be up when
 * we get there. And it warns a little BEFORE the limit (REACH_MARGIN_DEG): the
 * robot never stops exactly on its waypoint, and a target sitting right at the
 * limit means the turret is resting against its soft stop.
 *
 * Give it the RED version of the launch pose, like PathSafety.checkStart().
 * Blue is red spun 180 deg around the field center, and so is everything this
 * checks, so if red is OK, blue is too.
 *
 * NOT MEASURED YET? It still runs, on stand-ins, and says so in every warning:
 *   - turret offset unknown  -> turret on the robot's center
 *   - hive opening unknown   -> HiveTags' cell position (RED_HIVE_X is made-up)
 * The margin covers those guesses for now.
 */
public final class LaunchReach {

    private LaunchReach() {}

    public static double REACH_MARGIN_DEG = 10.0;

    /** Warnings for one launch spot (empty = the turret can reach the hive with margin). */
    public static List<String> check(String launchName, Pose redLaunch) {
        boolean geometryMeasured = TurretGeometry.forRobot().isConfigured();
        TurretGeometry geometry = geometryMeasured
                ? TurretGeometry.forRobot()
                : new TurretGeometry(0, 0, TurretGeometry.TURRET_ZERO_DEG, TurretGeometry.TURRET_CCW_SIGN);
        boolean hiveMeasured = HiveGeometry.isMeasured();

        String approx = "";
        if (!geometryMeasured || !hiveMeasured) {
            approx = " (approx: " + (geometryMeasured ? "" : "turret offset")
                    + (!geometryMeasured && !hiveMeasured ? ", " : "")
                    + (hiveMeasured ? "" : "hive opening") + " not measured)";
        }

        List<String> warnings = new ArrayList<>();
        for (Tilt tilt : Tilt.values()) {
            double[] target = hiveMeasured
                    ? HiveGeometry.aimPoint(Alliance.RED, tilt)
                    : approxRedCell(tilt);
            warnings.addAll(checkOne(launchName, redLaunch, geometry, target, tilt.toString(),
                    Turret.MIN_ANGLE, Turret.MAX_ANGLE, REACH_MARGIN_DEG, approx));
        }
        return warnings;
    }

    // Separate so tests can use made-up robots and targets.
    static List<String> checkOne(String launchName, Pose launch, TurretGeometry geometry,
                                 double[] target, String targetName,
                                 double minDeg, double maxDeg, double marginDeg, String approx) {
        List<String> warnings = new ArrayList<>();
        double angle = geometry.aimAngle(launch, target[0], target[1]);
        if (angle < minDeg || angle > maxDeg) {
            double past = angle > maxDeg ? angle - maxDeg : minDeg - angle;
            warnings.add(String.format("%s: turret CAN'T reach the hive (%s), %.0f deg past its limit%s",
                    launchName, targetName, past, approx));
        } else if (angle > maxDeg - marginDeg || angle < minDeg + marginDeg) {
            double room = Math.min(maxDeg - angle, angle - minDeg);
            warnings.add(String.format("%s: hive (%s) only %.0f deg inside the turret's limit%s",
                    launchName, targetName, room, approx));
        }
        return warnings;
    }

    /** Stand-in for the opening: the cell's center from HiveTags. Red hive, Pedro inches. */
    static double[] approxRedCell(Tilt tilt) {
        double y = tilt == Tilt.FAR_UP
                ? HiveTags.PIVOT_Y + HiveTags.CELL_ARM
                : HiveTags.PIVOT_Y - HiveTags.CELL_ARM;
        return new double[] { HiveTags.RED_HIVE_X, y };
    }
}
