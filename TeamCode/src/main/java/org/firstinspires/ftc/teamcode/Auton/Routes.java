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
 *
 * Using the Pedro Pathing visualizer:
 *   The visualizer is a SKETCHPAD, not a place to get code from. Don't paste
 *   its exported Java into this repo -- it's written for a newer Pedro than
 *   ours (it won't compile), it only knows red, and its numbers don't move
 *   when the robot size in RobotConstants changes. Instead:
 *     1. Sketch the route in the visualizer with an 18 x 18 robot.
 *     2. Read off the corner points. For each one: is it already in
 *        Waypoints, or is it a new place?
 *     3. Add new points to Waypoints, built from FieldConstants and
 *        RobotConstants where you can.
 *     4. Write the route here, and make the tiny OpModes.
 *     5. INIT it and read the path check on the Driver Station. The path
 *        check is the final word, not the picture in the visualizer.
 *   Routes drive straight lines and stop at each point. Curved, no-stop
 *   paths (like the visualizer draws) aren't supported yet -- ask before
 *   adding them, so it's done in Route/RouteAuto for every auto at once.
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

    // Start against the far wall, go around the left of the hive, drive along
    // the garden wall into the corner collecting the garden pollen, then back
    // up to the garden wall start point (where START_UP_GARDEN_WALL begins).
    public static final Route START_UP_FAR_WALL = Route.startAt(FAR_WALL_START)
            .driveTo(HIVE_TOP)
            .driveTo(HIVE_LEFT_HIGH)
            .driveTo(GARDEN_APPROACH)      // straight down the left of the hive to the wall
            .collectTo(GARDEN_CORNER)      // along the wall, collector on near the corner
            .driveTo(GARDEN_WALL_START);   // back up along the wall (still facing the red wall)
}
