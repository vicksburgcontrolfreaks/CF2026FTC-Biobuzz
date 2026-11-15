package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * 6494's park, 1 in. closer to our wall (Shane, 2026-11-15; was 3). RouteAuto moves
 * the LAST step of every route that marked a park by PARK_WALL_SHIFT_6494 toward
 * the red wall (before mirroring). This builds every competition route the same
 * way and checks the moved park still passes the path check, stays off the wall
 * and stays in the loading zone.
 */
public class ParkWallShiftTest {

    private static final double EPS = 1e-9;
    private static final double SHIFT = RouteAuto.PARK_WALL_SHIFT_6494;

    private static List<Route> competitionRoutes(Routes.Park p) {
        List<Route> r = new ArrayList<>();
        r.add(Routes.gardenShoot2X(p));
        r.add(Routes.gardenShoot2XTipCheck(p));
        r.add(Routes.farWallWaitShoot(p));
        r.add(Routes.gardenLeavePark2026(p));
        r.add(Routes.farLeavePark2026(p));
        return r;
    }

    @Test public void shiftIsThreeInchesTowardTheRedWall() {
        assertEquals(1.0, SHIFT, EPS);
        Pose p = Waypoints.towardRedWall(Waypoints.PARK_GARDEN_SIDE, SHIFT);
        assertEquals(Waypoints.PARK_GARDEN_SIDE.getX() - 1, p.getX(), EPS);
        assertEquals(Waypoints.PARK_GARDEN_SIDE.getY(), p.getY(), EPS);
        assertEquals(Waypoints.PARK_GARDEN_SIDE.getHeading(), p.getHeading(), EPS);
    }

    /** Built exactly like RouteAuto.buildPaths() on 6494: no path warnings anywhere. */
    @Test public void shiftedParkPassesThePathCheck() {
        for (Alliance a : Alliance.values()) {
            for (Routes.Park p : new Routes.Park[]{Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE}) {
                for (Route r : competitionRoutes(p)) {
                    assertTrue(r.parkFrom >= 0);
                    List<String> w = new ArrayList<>();
                    Pose from = a.map(r.start);
                    int last = r.steps.size() - 1;
                    for (int i = 0; i <= last; i++) {
                        Route.Step s = r.steps.get(i);
                        if (s.target == null) continue;
                        Pose red = i == last ? Waypoints.towardRedWall(s.target, SHIFT) : s.target;
                        Pose to = a.map(red);
                        Path path = new Path(new BezierLine(from, to));
                        path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
                        w.addAll(PathSafety.check(a + " " + p + " step " + (i + 1), path));
                        from = to;
                    }
                    assertTrue(a + " / " + p + ": " + w, w.isEmpty());
                }
            }
        }
    }

    /** Commanded wall gap 4 -> 1 in.: still OFF the wall, still in the zone. */
    @Test public void shiftedParkIsOffTheWallAndInTheZone() {
        for (Routes.Park p : new Routes.Park[]{Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE}) {
            Pose park = Waypoints.towardRedWall(p.spot, SHIFT);
            double edge = park.getX() - RobotConstants.ROBOT_LENGTH / 2;   // facing red: length along x
            assertEquals(p + " wall gap", Waypoints.PARK_WALL_GAP - SHIFT, edge, EPS);
            assertTrue(p + " touches the wall", edge > 0.5);
            double minY = park.getY() - RobotConstants.ROBOT_WIDTH / 2;
            double maxY = park.getY() + RobotConstants.ROBOT_WIDTH / 2;
            assertTrue(p + " left the loading zone",
                    maxY > FieldConstants.RED_LOADING_ZONE.minY && minY < FieldConstants.RED_LOADING_ZONE.maxY
                            && edge < FieldConstants.RED_LOADING_ZONE.maxX);
        }
    }
}
