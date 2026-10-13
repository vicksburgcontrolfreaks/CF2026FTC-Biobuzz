package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Auton.Waypoints.*;

/*
 * Every route our autos can drive, in one place. Written in RED coordinates
 * using the points in Waypoints -- blue is figured out automatically.
 *
 * Adding a route:
 *   1. Add any new points to Waypoints.
 *   2. Write the route here.
 *   3. Make a tiny OpMode for each alliance (copy RedStartUp.java, change the
 *      name, the alliance, and the route).
 */
public final class Routes {

    private Routes() {} // nobody should make a Routes object

    // Start against the garden wall, collect the garden pollen, then go
    // around the left of the hive to the far wall.
    public static final Route START_UP_GARDEN_WALL = Route.startAt(GARDEN_WALL_START)
            .collectTo(GARDEN_CORNER)    // along the wall, collector on near the corner
            .driveTo(HIVE_LEFT_LOW)
            .driveTo(HIVE_LEFT_HIGH)
            .driveTo(HIVE_TOP)
            .driveTo(FAR_WALL_START);

    // Start against the far wall, go around the left of the hive, then drive
    // along the garden wall into the corner, collecting the garden pollen.
    public static final Route START_UP_FAR_WALL = Route.startAt(FAR_WALL_START)
            .driveTo(HIVE_TOP)
            .driveTo(HIVE_LEFT_HIGH)
            .driveTo(GARDEN_APPROACH)    // straight down the left of the hive to the wall
            .collectTo(GARDEN_CORNER);   // along the wall, collector on near the corner
}
