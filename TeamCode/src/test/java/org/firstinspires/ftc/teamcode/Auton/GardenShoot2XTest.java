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
        assertEquals(Routes.SHOOT_2X_COLLECT_SPEED, collect.speed, EPS);
        assertEquals(Routes.SHOOT_2X_WALL_DWELL_SEC, collect.dwellSec, EPS);
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
     * The park no longer turns (Shane, 2026-11-14 evening): every park leg stays
     * at heading 180, collector toward red. Removing the in-motion 180 deg turn
     * is the test for the ~30 in. odometry error seen on the last run. TeleOp
     * needs nothing: field-centric uses the real heading.
     */
    @Test public void parksStillFacingRedNoTurn() {   // Shane, 2026-11-14 evening
        for (Routes.Park p : new Routes.Park[] {Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE}) {
            Route r = Routes.gardenShoot2X(p);
            Pose park = r.steps.get(r.steps.size() - 1).target;
            assertEquals(p + " park heading", Math.PI, park.getHeading(), EPS);
            for (int i = r.parkFrom; i < r.steps.size(); i++) {
                assertEquals("park leg " + i + " must not turn", Math.PI, r.steps.get(i).target.getHeading(), EPS);
            }
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

    /**
     * A BALL CANNOT STOP THE ROUTE ANY MORE (Shane, 5618, 2026-11-14): the sweep
     * wedged just short of the garden corner and the route never got past it,
     * because a step only ends when the follower says it arrived.
     *
     * Both wall legs need the limit, not just the sweep -- the drive back covers
     * the same 24 in. of the same wall. The launch steps must NOT have one: they
     * have no driving to time, and fireSec owns the firing.
     */
    @Test public void bothWallLegsGiveUpRatherThanPushForever() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        assertEquals("sweep", Routes.SHOOT_2X_SWEEP_TIMEOUT_SEC,
                r.steps.get(1).timeoutSec, EPS);
        assertEquals("drive back", Routes.SHOOT_2X_RETURN_TIMEOUT_SEC,
                r.steps.get(2).timeoutSec, EPS);
        for (int i : new int[] {0, 3}) {
            assertTrue("launch step " + i + " should have no drive timeout",
                    Double.isNaN(r.steps.get(i).timeoutSec));
        }
        // A limit shorter than the leg would cut off runs that were going to make
        // it. 24 in. at 0.2 power is not fast, but it is not 8 seconds slow either.
        assertTrue("sweep limit should be the longer one -- it drives slowly",
                Routes.SHOOT_2X_SWEEP_TIMEOUT_SEC > Routes.SHOOT_2X_RETURN_TIMEOUT_SEC);
    }

    /**
     * THE PARK IS MARKED, AND THE MARK COVERS ALL FOUR LEGS. RouteAuto bails out
     * to parkFrom at PARK_LEAVE_BY_SEC from anywhere earlier in the route, so a
     * mark that pointed at the park spot alone would send the robot down the
     * diagonal that hit the hive on 2026-11-14 -- the three legs before it are
     * the clearance.
     */
    @Test public void theWholeParkIsMarkedAsThePark() {
        Route r = Routes.gardenShoot2X(Routes.Park.GARDEN_SIDE);
        assertEquals("park starts right after the second shot", 4, r.parkFrom);
        assertEquals("four park legs", 4, r.steps.size() - r.parkFrom);
        // The second shot is the last thing before the park, so nothing that
        // scores gets skipped by a bail-out that fires early.
        assertEquals(Route.Action.LAUNCH_HERE, r.steps.get(r.parkFrom - 1).action);
        for (int i = r.parkFrom; i < r.steps.size(); i++) {
            assertEquals("park leg " + i + " just drives",
                    Route.Action.DRIVE, r.steps.get(i).action);
        }
        // Bailing out to the first park leg means driving to it from wherever we
        // are, which is somewhere on the garden wall between the corner and the
        // shot spot. It is on that same wall, so that is a straight line along
        // it, not a diagonal across the field.
        assertEquals("first park leg stays on the garden wall",
                r.start.getY(), r.steps.get(r.parkFrom).target.getY(), EPS);
    }

    /** "Don't park" has no park to bail out to, and says so instead of guessing. */
    @Test public void dontParkHasNoParkMark() {
        assertEquals(-1, Routes.gardenShoot2X(Routes.Park.NONE).parkFrom);
    }

    /** timeout() changes only the step it follows, and nothing else about it. */
    @Test public void timeoutOnlyTouchesTheStepBeforeIt() {
        Route r = Route.startAt(Waypoints.GARDEN_EDGE_START)
                .collectTo(Waypoints.GARDEN_CORNER, 0.2, 2).timeout(8)
                .driveTo(Waypoints.GARDEN_EDGE_START);
        assertEquals(8.0, r.steps.get(0).timeoutSec, EPS);
        assertEquals(0.2, r.steps.get(0).speed, EPS);        // speed survives
        assertEquals(2.0, r.steps.get(0).dwellSec, EPS);     // so does the dwell
        assertEquals(Route.Action.COLLECT, r.steps.get(0).action);
        assertTrue("the next step is untouched", Double.isNaN(r.steps.get(1).timeoutSec));
    }

    /** beginPark() marks a position, not a step -- the step list is unchanged. */
    @Test public void beginParkAddsNoStep() {
        Route plain = Route.startAt(Waypoints.GARDEN_EDGE_START)
                .driveTo(Waypoints.GARDEN_CORNER);
        Route marked = Route.startAt(Waypoints.GARDEN_EDGE_START)
                .beginPark()
                .driveTo(Waypoints.GARDEN_CORNER);
        assertEquals(plain.steps.size(), marked.steps.size());
        assertEquals(0, marked.parkFrom);
        assertEquals(-1, plain.parkFrom);
    }
}
