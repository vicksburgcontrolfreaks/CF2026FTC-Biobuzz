package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.Zone;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Garden Shoot 2X (Shane, 2026-11-14): shoot where we stand, sweep the garden
 * wall, come back to the same spot, shoot again, park facing blue.
 *
 * The path check is the same one RouteAuto runs at INIT, but here it fails on
 * the laptop instead of as a Driver Station warning nobody reads under pressure.
 * The rest of these tests pin the things that are easy to get backwards and
 * expensive to find on a robot: the start heading, that the collector LEADS down
 * the wall, that going back to the start never builds a zero-length path, and
 * that the park turns the robot round for the drivers.
 */
public class GardenShoot2XTest {

    private static final double EPS = 1e-6;

    /**
     * Every path-safety warning for a route, built the way RouteAuto builds it.
     * LAUNCH_HERE steps have a NULL target and no path, so they contribute
     * nothing and do not move `from` -- same as in RouteAuto.buildPaths().
     */
    private static List<String> warnings(Route route, Alliance alliance, Routes.Park park) {
        List<String> w = new ArrayList<>(PathSafety.checkStart("Start", route.start));
        Pose from = alliance.map(route.start);
        int i = 1;
        for (Route.Step s : route.steps) {
            if (s.action == Route.Action.LAUNCH_HERE) {
                i++;
                continue;
            }
            Pose to = alliance.map(s.target);
            Path path = new Path(new BezierLine(from, to));
            path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
            w.addAll(PathSafety.check(alliance + " " + park + " step " + i++, path));
            from = to;
        }
        return w;
    }

    @Test public void noWarningsEitherAllianceEitherPark() {
        for (Alliance a : Alliance.values()) {
            for (Routes.Park p : Routes.Park.values()) {
                Route r = Routes.gardenShoot2X(p);
                List<String> w = warnings(r, a, p);
                assertTrue(a + " / " + p + ": " + w, w.isEmpty());
            }
        }
    }

    @Test public void startsOnTheSeamWithTheCollectorFacingRed() {
        Pose start = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE).start;
        // Same CENTRE as Garden Shoot + Park's start -- there the back edge is on
        // the seam, here the collector edge is, and both put the centre half a
        // robot length past x = 24.
        assertEquals(33.0, start.getX(), EPS);
        assertEquals(9.0, start.getY(), EPS);   // touching the garden (audience) wall
        // 180 = collector toward the RED wall, which puts the turret on the hive side.
        assertEquals(180.0, Math.toDegrees(start.getHeading()), EPS);
        assertEquals(Waypoints.GARDEN_EDGE_START.getX(), start.getX(), EPS);
        assertEquals(Waypoints.GARDEN_EDGE_START.getY(), start.getY(), EPS);
    }

    @Test public void shootsCollectsReturnsThenShootsAgain() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        assertEquals(Route.Action.LAUNCH_HERE, r.steps.get(0).action);
        assertEquals(Route.Action.COLLECT, r.steps.get(1).action);
        assertEquals(Route.Action.DRIVE, r.steps.get(2).action);
        assertEquals(Route.Action.LAUNCH_HERE, r.steps.get(3).action);
        assertEquals(8, r.steps.size());   // + wall-back + turn-back + past-flower + park
    }

    /**
     * REAL CLEARANCE, not just "does not overlap". The first version of this
     * route passed PathSafety and still hit the hive on the robot: the commanded
     * line cleared it by about 7 in., which is less than 6494's measured pose
     * drift (docs/TODO.md item 0). So the laptop check has to enforce a margin
     * that survives the drift, or it is checking the wrong thing.
     *
     * The hive is the one we hit, so it gets the big margin. The flowers are
     * small, wall-mounted and only ever passed alongside, so they get the 3 in.
     * the repo already uses (PAST_FLOWER_CLEARANCE).
     */
    @Test public void everyLegKeepsRealClearanceFromTheHiveAndFlowers() {
        for (Alliance a : Alliance.values()) {
            for (Routes.Park p : Routes.Park.values()) {
                Route r = Routes.gardenShoot2X(p);
                assertClearance(r, a, p, FieldConstants.HIVE, 12.0);
                for (Zone flower : FieldConstants.FLOWERS) {
                    assertClearance(r, a, p, flower, 3.0);
                }
            }
        }
    }

    private static void assertClearance(Route route, Alliance alliance, Routes.Park park,
                                        Zone obstacle, double minGap) {
        double worst = Double.POSITIVE_INFINITY;
        Pose worstAt = null;
        Pose from = alliance.map(route.start);
        for (Route.Step s : route.steps) {
            if (s.action == Route.Action.LAUNCH_HERE) continue;   // no path, no motion
            Pose to = alliance.map(s.target);
            Path path = new Path(new BezierLine(from, to));
            path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
            int steps = Math.max(10, (int) Math.ceil(path.length()));
            for (int i = 0; i <= steps; i++) {
                double t = (double) i / steps;
                Pose at = path.getPoint(t);
                double gap = gap(footprint(at.getX(), at.getY(), path.getHeadingGoal(t)), obstacle);
                if (gap < worst) {
                    worst = gap;
                    worstAt = at;
                }
            }
            from = to;
        }
        assertTrue(String.format("%s / %s: only %.1f in. from %s near (%.0f, %.0f) -- want %.0f",
                        alliance, park, worst, obstacle.name,
                        worstAt == null ? -1 : worstAt.getX(), worstAt == null ? -1 : worstAt.getY(),
                        minGap),
                worst >= minGap);
    }

    /**
     * Inches between two boxes: the largest separation on any one axis, which is
     * what "they do not overlap" means. Zero or less = touching or overlapping.
     */
    private static double gap(Zone robot, Zone obstacle) {
        return Math.max(
                Math.max(obstacle.minX - robot.maxX, robot.minX - obstacle.maxX),
                Math.max(obstacle.minY - robot.maxY, robot.minY - obstacle.maxY));
    }

    /** The wall-aligned box the robot covers, same as PathSafety's. */
    private static Zone footprint(double x, double y, double heading) {
        double cos = Math.abs(Math.cos(heading));
        double sin = Math.abs(Math.sin(heading));
        double halfX = (RobotConstants.ROBOT_LENGTH * cos + RobotConstants.ROBOT_WIDTH * sin) / 2;
        double halfY = (RobotConstants.ROBOT_LENGTH * sin + RobotConstants.ROBOT_WIDTH * cos) / 2;
        return new Zone("Robot", x - halfX, x + halfX, y - halfY, y + halfY);
    }

    /** The turn-back is at least a foot toward red of the shot spot (Shane's call). */
    @Test public void turnBackMovesAtLeastAFootTowardRed() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        Pose turnBack = r.steps.get(5).target;
        assertTrue("turn-back should be >= 12 in. toward red of the shot spot",
                r.start.getX() - turnBack.getX() >= 12.0);
        // And it must not have traded the hive for the red-wall flower.
        assertTrue("turn-back should clear the red-wall flower",
                turnBack.getX() - RobotConstants.ROBOT_LENGTH / 2
                        - FieldConstants.FLOWER_RED_WALL.maxX >= 3.0);
    }

    @Test public void bothLaunchesFireFourSecondsAndHaveNoTarget() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        for (int i : new int[] {0, 3}) {
            Route.Step s = r.steps.get(i);
            assertEquals(Routes.SHOOT_2X_FIRE_SEC, s.fireSec, EPS);
            // No path, so nothing to aim the follower at -- and nothing that could
            // become a zero-length path when we shoot from a spot we drove to.
            assertNull("step " + i + " should have no target", s.target);
        }
    }

    @Test public void collectLegIsSlowDwellsAndLeadsWithTheCollector() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        Route.Step collect = r.steps.get(1);
        assertEquals(0.2, collect.speed, EPS);
        assertEquals(2.0, collect.dwellSec, EPS);
        // Into the garden corner: along the wall (y unchanged), toward the red
        // wall (x down). Heading 180 means the collector points down-x, so the
        // robot drives collector-first and the balls go in rather than past.
        Pose start = r.start;
        assertEquals(start.getY(), collect.target.getY(), EPS);
        assertTrue("should travel toward the red wall",
                collect.target.getX() < start.getX());
        assertEquals(180.0, Math.toDegrees(collect.target.getHeading()), EPS);
    }

    @Test public void returnsToExactlyTheStartPoseSoTheSecondShotIsTheSameShot() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        Pose back = r.steps.get(2).target;
        assertEquals(r.start.getX(), back.getX(), EPS);
        assertEquals(r.start.getY(), back.getY(), EPS);
        assertEquals(r.start.getHeading(), back.getHeading(), EPS);
    }

    /**
     * The park turns the robot 180 deg so the COLLECTOR faces blue. For red that
     * is heading 0, which is also Alliance.RED.awayFromDriversHeading() -- so
     * TeleOp's field-centric forward is collector-first away from the drivers
     * with driverHeadingOffset left at 0, and nothing has to edit the pose.
     */
    @Test public void parksFacingBlueForTheTeleOpHandoff() {
        for (Routes.Park p : new Routes.Park[] {Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE}) {
            Route r = Routes.gardenShoot2X(p);
            Pose park = r.steps.get(r.steps.size() - 1).target;
            assertEquals(p + " park heading", 0.0, park.getHeading(), EPS);
            assertEquals(p + " park spot", p.spot.getX(), park.getX(), EPS);
            assertEquals(p + " park spot", p.spot.getY(), park.getY(), EPS);
        }
        assertEquals(0.0, Alliance.RED.awayFromDriversHeading(), EPS);
    }

    /** "Don't park" stops after the second shot, still facing red. */
    @Test public void dontParkEndsAtTheShotSpot() {
        Route r = Routes.gardenShoot2X(Routes.Park.NONE);
        assertEquals(4, r.steps.size());
        assertEquals(Route.Action.LAUNCH_HERE, r.steps.get(r.steps.size() - 1).action);
    }

    /** Only the collect leg is slow; the drive back is at the route's normal speed. */
    @Test public void onlyTheCollectLegIsSlow() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        assertTrue("drive back should use the route speed",
                Double.isNaN(r.steps.get(2).speed));
        assertTrue("park should use the route speed",
                Double.isNaN(r.steps.get(7).speed));
    }
}
