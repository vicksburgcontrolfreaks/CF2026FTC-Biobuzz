package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Garden Shoot + Park (Shane, 2026-11-06): the start is legal and no leg hits
 * the hive, a flower or a wall -- same check RouteAuto runs at INIT, but here
 * it fails on the laptop instead of as a DS warning.
 */
public class GardenShootParkTest {

    private static final double EPS = 1e-6;

    /** Every path-safety warning for this route, built the way RouteAuto builds it. */
    private static List<String> warnings(Alliance alliance, Routes.Park park) {
        return warnings(Routes.gardenShootPark(park), alliance, park);
    }

    private static List<String> warnings(Route route, Alliance alliance, Routes.Park park) {
        List<String> w = new ArrayList<>(PathSafety.checkStart("Start", route.start));
        Pose from = alliance.map(route.start);
        int i = 1;
        for (Route.Step s : route.steps) {
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
                List<String> w = warnings(a, p);
                assertTrue(a + " / " + p + ": " + w, w.isEmpty());
            }
        }
    }

    @Test public void startsJustOutsideTheGardenFacingBlue() {
        Pose start = Routes.gardenShootPark(Routes.Park.GARDEN_SIDE).start;
        assertEquals(33.0, start.getX(), EPS);   // back edge on x = 24 (garden tape end)
        assertEquals(9.0, start.getY(), EPS);    // touching the audience wall
        assertEquals(0.0, start.getHeading(), EPS);
    }

    @Test public void launchesFirstThenParksFacingBlue() {
        Route route = Routes.gardenShootPark(Routes.Park.GARDEN_SIDE);
        assertEquals(Route.Action.LAUNCH, route.steps.get(0).action);
        Pose shot = route.steps.get(0).target;
        assertEquals(57.0, shot.getX(), EPS);
        assertEquals(27.0, shot.getY(), EPS);
        assertEquals(Math.toDegrees(Math.atan2(72 - 27, 72 - 57)), Math.toDegrees(shot.getHeading()), EPS);
        Pose park = route.steps.get(route.steps.size() - 1).target;
        assertEquals(0.0, park.getHeading(), EPS);
    }

    @Test public void backupLeaveParkIsClearAndFacesBlue() {
        for (Alliance a : Alliance.values()) {
            for (Routes.Park p : Routes.Park.values()) {
                Route r = Routes.gardenLeavePark2026(p);
                List<String> w = warnings(r, a, p);
                assertTrue("backup " + a + " / " + p + ": " + w, w.isEmpty());
                assertEquals(1, r.steps.size());   // straight to the park, NONE parks garden side
                assertEquals(0.0, r.steps.get(0).target.getHeading(), EPS);
            }
        }
    }

    @Test public void dontParkStopsAtTheLaunchSpot() {
        assertEquals(1, Routes.gardenShootPark(Routes.Park.NONE).steps.size());
    }
}
