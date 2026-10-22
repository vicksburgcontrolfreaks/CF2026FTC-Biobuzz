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
        return route.driveTo(HIVE_LEFT_HIGH).driveTo(park.spot);
    }

    // The no-park versions, for the old one-route OpModes (RedStartUp etc.).
    public static final Route START_UP_GARDEN_WALL = startUpGardenWall(Park.NONE);
    public static final Route START_UP_FAR_WALL    = startUpFarWall(Park.NONE);
}
