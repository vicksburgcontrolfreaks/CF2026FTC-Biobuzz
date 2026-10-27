package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/*
 * Tests for LaunchReach. Run with:  gradlew :TeamCode:testDebugUnitTest
 * checkOne() gets made-up robots and targets; check() is run against today's
 * real (partly made-up) constants only to prove it runs and labels guesses.
 */
public class LaunchReachTest {

    private static final double MIN = -140, MAX = 140, MARGIN = 10;
    private static final TurretGeometry SIMPLE = new TurretGeometry(0, 0, 0, +1);
    private static final double[] TARGET = { 72, 72 };

    private static List<String> at(double x, double y, double headingDeg) {
        return LaunchReach.checkOne("Test spot", new Pose(x, y, Math.toRadians(headingDeg)),
                SIMPLE, TARGET, "cell", MIN, MAX, MARGIN, "");
    }

    @Test public void targetToTheSideIsFine() {
        assertTrue(at(72, 24, 0).isEmpty());     // target dead left: turret +90
        assertTrue(at(72, 120, 0).isEmpty());    // dead right: -90
        assertTrue(at(24, 72, 0).isEmpty());     // straight ahead: 0
    }

    @Test public void targetBehindCantBeReached() {
        List<String> w = at(120, 72, 0);         // target straight behind: 180
        assertEquals(1, w.size());
        assertTrue(w.get(0), w.get(0).contains("CAN'T reach"));
        assertTrue(w.get(0), w.get(0).contains("40 deg past"));
    }

    @Test public void closeToTheLimitWarns() {
        // Target at 135 deg: inside the 140 limit, but within the 10 deg margin.
        double h = Math.toRadians(135);
        List<String> w = at(72 - 40 * Math.cos(h), 72 - 40 * Math.sin(h), 0);
        assertEquals(1, w.size());
        assertTrue(w.get(0), w.get(0).contains("only 5 deg inside"));
    }

    @Test public void justOutsideTheMarginIsQuiet() {
        double h = Math.toRadians(-125);
        assertTrue(at(72 - 40 * Math.cos(h), 72 - 40 * Math.sin(h), 0).isEmpty());
    }

    @Test public void turningTheRobotFixesIt() {
        // Same spot as targetBehindCantBeReached, robot turned around.
        assertTrue(at(120, 72, 180).isEmpty());
    }

    @Test public void approxLabelIsCarriedThrough() {
        List<String> w = LaunchReach.checkOne("Spot", new Pose(120, 72, 0), SIMPLE, TARGET, "cell",
                MIN, MAX, MARGIN, " (approx)");
        assertTrue(w.get(0).endsWith(" (approx)"));
    }

    @Test public void realCheckRunsOnStandInsAndSaysSo() {
        // Robot facing the red wall with the hive behind it: unreachable for both tilts.
        // Only valid while the hive opening is unmeasured -- then it uses stand-ins.
        org.junit.Assume.assumeTrue(!HiveGeometry.isMeasured());
        List<String> w = LaunchReach.check("Behind", new Pose(130, 72, Math.toRadians(0)));
        assertEquals(2, w.size());
        for (String s : w) assertTrue(s, s.contains("approx"));
    }
}
