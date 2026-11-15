package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.PathSafety;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/** Far Wall Wait and Shoot (Shane, 2026-11-14): laptop checks before the robot. */
public class FarWallWaitShootTest {

    private static final double EPS = 1e-6;
    private static final Routes.Park[] PARKS = {Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE};

    @Test public void startsOnTheFarWallFacingRedWithTheCollectorOnTheSeam() {
        Pose s = Routes.farWallWaitShoot(Routes.Park.FAR_SIDE).start;
        assertEquals(33, s.getX(), EPS);   // collector edge at x = 24, the first seam
        assertEquals(135, s.getY(), EPS);  // side touching the far wall
        assertEquals(180, Math.toDegrees(s.getHeading()), EPS);
        assertTrue(PathSafety.checkStart("Start", s).isEmpty());
    }

    @Test public void waitsForTheTipWithoutMovingThenParks() {
        for (Routes.Park p : PARKS) {
            Route r = Routes.farWallWaitShoot(p);
            assertEquals(2, r.steps.size());
            Route.Step tip = r.steps.get(0);
            assertEquals(Route.Action.TIP_LAUNCH_HERE, tip.action);
            assertNull(tip.target);                     // no path: shoots where it stands
            assertEquals(5.0, tip.fireSec, EPS);
            assertTrue(tip.cutForPark);                 // 25 s cut-off applies
            assertEquals(1, r.parkFrom);                // 25 s deadline jumps to the park
            Pose park = r.steps.get(1).target;
            assertEquals(p.spot.getX(), park.getX(), EPS);
            assertEquals(p.spot.getY(), park.getY(), EPS);
            assertEquals(Math.PI, park.getHeading(), EPS);   // no turn
        }
    }

    @Test public void alwaysParksEvenIfAskedNotTo() {
        Route r = Routes.farWallWaitShoot(Routes.Park.NONE);
        assertTrue(r.parkFrom >= 0);
        assertEquals(2, r.steps.size());
    }

    @Test public void bothParksArePathSafeOnBothAlliances() {
        for (Alliance a : Alliance.values()) {
            for (Routes.Park p : PARKS) {
                Route r = Routes.farWallWaitShoot(p);
                Pose from = a.map(r.start);
                Pose to = a.map(r.steps.get(1).target);
                Path path = new Path(new BezierLine(from, to));
                path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
                List<String> w = PathSafety.check(a + " " + p, path);
                assertTrue(w.toString(), w.isEmpty());
            }
        }
    }

    @Test public void blueIsMirrored() {
        Pose b = Alliance.BLUE.map(Routes.farWallWaitShoot(Routes.Park.FAR_SIDE).start);
        assertEquals(111, b.getX(), EPS);   // blue scoring side, audience wall
        assertEquals(9, b.getY(), EPS);
        assertEquals(0, b.getHeading(), EPS);   // collector toward the blue wall
    }
}
