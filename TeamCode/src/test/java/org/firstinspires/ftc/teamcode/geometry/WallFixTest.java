package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/*
 * WallFix on a laptop. Every test runs the camera FORWARDS from a robot pose we
 * choose (what bearing would it see to each tag?), feeds those bearings to
 * WallFix, and checks it lands back on the pose.
 *
 * The robot is 6494-shaped: turret axis 4.5 in behind centre, camera 4.5 in
 * ahead of the axis. The tags are where the red hive's audience cell roughly is
 * (x ~ 53-66, y ~ 58). Exact hive numbers do not matter here -- the forward
 * model and the solve use the same ones.
 */
public class WallFixTest {

    private static final double EPS = 1e-6;
    private static final double CAM_FWD = 4.5;
    private static final double CAM_LEFT = 0.0;
    private static final TurretGeometry GEO = new TurretGeometry(-4.5, 0.0, 0.0, 1.0);

    private static final double[][] RED_AUDIENCE_TAGS = {
            {53.1, 58.0}, {56.85, 58.0}, {62.35, 58.0}, {66.1, 58.0}};

    /** What the camera would report: turret-frame bearing to each tag, + = left. */
    private static List<WallFix.Sighting> see(Pose robot, double turretDeg, double[][] tags) {
        Pose axis = GEO.turretPosition(robot);
        double phi = GEO.turretFieldHeading(robot, turretDeg);
        double cx = axis.getX() + CAM_FWD * Math.cos(phi) - CAM_LEFT * Math.sin(phi);
        double cy = axis.getY() + CAM_FWD * Math.sin(phi) + CAM_LEFT * Math.cos(phi);
        List<WallFix.Sighting> out = new ArrayList<>();
        for (double[] t : tags) {
            double field = Math.atan2(t[1] - cy, t[0] - cx);
            double bearing = TurretGeometry.wrapDegrees(Math.toDegrees(field - phi));
            out.add(new WallFix.Sighting(t[0], t[1], bearing));
        }
        return out;
    }

    /** Turret angle that points roughly at the middle of the tags. */
    private static double aimAt(Pose robot, double[][] tags) {
        double x = 0, y = 0;
        for (double[] t : tags) { x += t[0]; y += t[1]; }
        return GEO.aimAngle(robot, x / tags.length, y / tags.length);
    }

    private static WallFix.Result solve(Pose truth, double odometryHeadingRad,
                                        WallFix.Face face, double[][] tags) {
        double turret = aimAt(truth, tags);
        return WallFix.solve(odometryHeadingRad, face, 9.0, turret, GEO, CAM_FWD, CAM_LEFT,
                see(truth, turret, tags));
    }

    // ---- bearingDeg ---------------------------------------------------------

    @Test
    public void bearingUndoesThePitch() {
        // A point 40 in ahead, 10 in left, 30 in up of the camera, in the
        // TURRET's level frame. Express it in a camera pitched up 38 deg.
        double f = 40, l = 10, u = 30, p = Math.toRadians(38);
        double camX = -l;
        double camY = f * Math.cos(p) + u * Math.sin(p);
        double camZ = -f * Math.sin(p) + u * Math.cos(p);
        assertEquals(Math.toDegrees(Math.atan2(l, f)), WallFix.bearingDeg(camX, camY, camZ, 38, 0), EPS);
    }

    @Test
    public void bearingAddsCameraYaw() {
        // Dead ahead of a camera turned 3 deg left is 3 deg left of the turret.
        assertEquals(3.0, WallFix.bearingDeg(0, 50, 0, 0, 3), EPS);
    }

    // ---- the four wall starts -----------------------------------------------

    @Test
    public void redAudienceStartRightSideOnWall() {
        Pose truth = new Pose(33, 9, 0);
        WallFix.Result r = solve(truth, 0, WallFix.Face.RIGHT, RED_AUDIENCE_TAGS);
        assertTrue(r.why, r.ok());
        assertEquals(WallFix.Wall.AUDIENCE, r.wall);
        assertTrue(r.alongX());
        assertEquals(33, r.robot.getX(), EPS);
        assertEquals(9, r.robot.getY(), EPS);
        assertEquals(4, r.tagsUsed);
        assertEquals(0, r.spread, EPS);
    }

    @Test
    public void blueScoringStartIsTheMirror() {
        // Blue scoring start = red audience spun 180 about the centre, and so are its tags.
        double[][] tags = new double[RED_AUDIENCE_TAGS.length][];
        for (int i = 0; i < tags.length; i++) {
            tags[i] = new double[]{144 - RED_AUDIENCE_TAGS[i][0], 144 - RED_AUDIENCE_TAGS[i][1]};
        }
        Pose truth = new Pose(111, 135, Math.PI);
        WallFix.Result r = solve(truth, Math.PI, WallFix.Face.RIGHT, tags);
        assertTrue(r.why, r.ok());
        assertEquals(WallFix.Wall.SCORING, r.wall);
        assertEquals(111, r.robot.getX(), EPS);
        assertEquals(135, r.robot.getY(), EPS);
    }

    @Test
    public void collectorFirstIntoTheRedWall() {
        // Facing the red wall (180), front on it: x = 9, y is the unknown.
        Pose truth = new Pose(9, 40, Math.PI);
        double[][] tags = {{59.6, 58}, {59.6, 86}};
        WallFix.Result r = solve(truth, Math.PI, WallFix.Face.FRONT, tags);
        assertTrue(r.why, r.ok());
        assertEquals(WallFix.Wall.RED, r.wall);
        assertFalse(r.alongX());
        assertEquals(9, r.robot.getX(), EPS);
        assertEquals(40, r.robot.getY(), EPS);
    }

    // ---- the heading snap ---------------------------------------------------

    @Test
    public void driftedOdometryHeadingStillSnapsSquare() {
        // The robot really is square; odometry thinks it is 6 deg off.
        Pose truth = new Pose(40, 9, 0);
        WallFix.Result r = solve(truth, Math.toRadians(6), WallFix.Face.RIGHT, RED_AUDIENCE_TAGS);
        assertTrue(r.why, r.ok());
        assertEquals(-6, r.snapDeg, EPS);
        assertEquals(40, r.robot.getX(), EPS);
        assertEquals(0, r.robot.getHeading(), EPS);
    }

    @Test
    public void refusesWhenOdometryIsFarFromSquare() {
        Pose truth = new Pose(40, 9, 0);
        WallFix.Result r = solve(truth, Math.toRadians(25), WallFix.Face.RIGHT, RED_AUDIENCE_TAGS);
        assertFalse(r.ok());
        assertNull(r.robot);
    }

    // ---- refusals -----------------------------------------------------------

    @Test
    public void refusesASightLineAlongTheWall() {
        // On the red wall facing it, the turret axis is 4.5 in BEHIND centre, so
        // at x = 13.5. A tag at x = 13.5 puts the sight line dead along +y.
        Pose truth = new Pose(9, 20, Math.PI);
        double[][] tags = {{13.5, 80}};
        WallFix.Result r = solve(truth, Math.PI, WallFix.Face.FRONT, tags);
        assertFalse(r.ok());
    }

    @Test
    public void refusesWithNoTags() {
        WallFix.Result r = WallFix.solve(0, WallFix.Face.RIGHT, 9, 0, GEO, CAM_FWD, CAM_LEFT,
                Collections.<WallFix.Sighting>emptyList());
        assertFalse(r.ok());
        assertEquals("no tags", r.why);
    }

    // ---- sensitivity --------------------------------------------------------

    @Test
    public void oneDegreeOfBearingBiasMovesItAboutInchesPerDeg() {
        Pose truth = new Pose(33, 9, 0);
        double turret = aimAt(truth, RED_AUDIENCE_TAGS);
        List<WallFix.Sighting> clean = see(truth, turret, RED_AUDIENCE_TAGS);
        List<WallFix.Sighting> biased = new ArrayList<>();
        for (WallFix.Sighting s : clean) {
            biased.add(new WallFix.Sighting(s.tagX, s.tagY, s.bearingDeg + 1.0));
        }
        WallFix.Result ok = WallFix.solve(0, WallFix.Face.RIGHT, 9, turret, GEO, CAM_FWD, CAM_LEFT, clean);
        WallFix.Result off = WallFix.solve(0, WallFix.Face.RIGHT, 9, turret, GEO, CAM_FWD, CAM_LEFT, biased);
        double moved = Math.abs(off.robot.getX() - ok.robot.getX());
        // The linear estimate should be within ~15% of what a real degree does.
        assertEquals(ok.inchesPerDeg, moved, 0.15 * ok.inchesPerDeg);
        // And stays in the "about an inch or two" range claimed for this spot.
        assertTrue("in/deg " + ok.inchesPerDeg, ok.inchesPerDeg > 0.5 && ok.inchesPerDeg < 3.0);
    }
}
