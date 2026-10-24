package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Auton.Waypoints.*;

/*
 * Every route our autos can drive, in one place. Written in RED coordinates
 * using the points in Waypoints -- blue is figured out automatically.
 *
 * Adding a route:
 *   1. Add any new points to Waypoints.
 *   2. Write the route here.
 *   3. Add it to SelectAuto's menu (AutoRoute).
 */
public final class Routes {

    private Routes() {} // nobody should make a Routes object

    // Where to end auto. Talk to your alliance partner -- if they park too,
    // take opposite ends of the loading zone.
    public enum Park {
        GARDEN_SIDE("Garden side", PARK_GARDEN_SIDE),
        FAR_SIDE("Far side", PARK_FAR_SIDE),
        NONE("Don't park", null);

        final String label;
        final Pose spot;

        Park(String label, Pose spot) {
            this.label = label;
            this.spot = spot;
        }

        @Override
        public String toString() {
            return label; // what the menu shows
        }
    }

    // Start against the garden wall and collect the garden pollen, then go up
    // the left of the hive. Then either:
    //   - park: the loading zone is right there, so turn off to it, or
    //   - no park: keep going over the hive to the far wall.
    public static Route startUpGardenWall(Park park) {
        Route route = Route.startAt(GARDEN_WALL_START)
                .collectTo(GARDEN_CORNER)    // along the wall, collector on near the corner
                .driveTo(HIVE_LEFT_LOW)
                .driveTo(HIVE_LEFT_HIGH);
        if (park == Park.NONE) {
            return route.driveTo(HIVE_TOP).driveTo(FAR_WALL_START);
        }
        return route.driveTo(park.spot);
    }

    // Start against the far wall, go around the left of the hive, and drive
    // along the garden wall into the corner collecting the garden pollen.
    // Then either:
    //   - park: back out past the red-wall flower (HIVE_LEFT_LOW) and up
    //     to the loading zone, or
    //   - no park: back up along the wall to the garden wall start.
    public static Route startUpFarWall(Park park) {
        Route route = Route.startAt(FAR_WALL_START)
                .driveTo(HIVE_TOP)
                .driveTo(HIVE_LEFT_HIGH)
                .driveTo(GARDEN_APPROACH)      // straight down the left of the hive to the wall
                .collectTo(GARDEN_CORNER);     // along the wall, collector on near the corner
        if (park == Park.NONE) {
            return route.driveTo(GARDEN_WALL_START); // still facing the red wall
        }
        return route.driveTo(HIVE_LEFT_LOW).driveTo(park.spot);
    }

    // Start against the garden wall, launch the 4 preloads, collect the garden
    // pollen, go around the left of the hive to the far wall, launch again,
    // then park. Launching is a PLACEHOLDER until the launcher is built -- the
    // robot just sits still for the launch time (see RouteAuto).
    public static Route gardenWallLaunch(Park park) {
        Route route = Route.startAt(GARDEN_WALL_START)
                .launchFrom(LAUNCH_GARDEN_WALL)  // launch the preloads
                .driveTo(GARDEN_APPROACH)        // back to the wall so we collect along it
                .collectTo(GARDEN_CORNER)        // collector on near the corner
                .driveTo(HIVE_LEFT_LOW)
                .driveTo(HIVE_LEFT_HIGH)
                .driveTo(HIVE_TOP)
                .launchFrom(LAUNCH_FAR_WALL);    // launch what we collected
        if (park == Park.NONE) {
            return route;
        }
        return route
                .driveTo(FAR_WALL_EXIT)     // straight off the wall first, or we hit the flower
                .driveTo(HIVE_LEFT_HIGH)
                .driveTo(park.spot);
    }

    // A short auto that stays out of our partner's way. Start against the far
    // wall, back straight off it (LEAVE) and launch the preloads, slide over
    // under the flower into the red/far wall corner, wait there until the
    // "yield until" time, then park. Launching is a PLACEHOLDER (see RouteAuto).
    public static Route farWallYield(Park park) {
        Route route = Route.startAt(FAR_WALL_START)
                .launchFrom(FAR_WALL_EXIT)   // straight off the wall, then launch
                .driveTo(PAST_FAR_FLOWER)    // sideways, staying well below the flower
                .yieldAt(FAR_CORNER);        // wait here for our partner
        if (park == Park.NONE) {
            return route;
        }
        return route.driveTo(park.spot);     // straight down the red wall
    }

    // BACKUP autos: just LEAVE + PARK (about 8 points), no launching or
    // collecting. For when the launcher is broken, or a partner needs us to
    // stay out of the way. Use the Wait setting to start later.
    //
    // Unlike the other routes, "Don't park" makes no sense here, so these
    // always park. SelectAuto doesn't offer "Don't park" for them; if
    // something else passes Park.NONE anyway, they park on the garden side.

    // Along the wall to beside the hive, straight up, then park.
    // (A straight diagonal from the start would clip the hive.)
    public static Route gardenWallLeavePark(Park park) {
        return Route.startAt(GARDEN_WALL_START)
                .driveTo(GARDEN_APPROACH)
                .driveTo(HIVE_LEFT_LOW)
                .driveTo(parkSpot(park));
    }

    // Straight off the wall, slide under the flower, then park.
    public static Route farWallLeavePark(Park park) {
        return Route.startAt(FAR_WALL_START)
                .driveTo(FAR_WALL_EXIT)      // straight off the wall, or we hit the flower
                .driveTo(PAST_FAR_FLOWER)
                .driveTo(parkSpot(park));
    }

    private static Pose parkSpot(Park park) {
        return park == Park.NONE ? PARK_GARDEN_SIDE : park.spot;
    }
}
