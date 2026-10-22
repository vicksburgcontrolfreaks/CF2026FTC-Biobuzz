package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
 * A route = where the robot starts, plus a list of steps. Each step is
 * "drive to this point" and what to do on the way. Build one like this:
 *
 *     Route.startAt(Waypoints.GARDEN_WALL_START)
 *          .collectTo(Waypoints.GARDEN_CORNER)
 *          .driveTo(Waypoints.HIVE_LEFT_LOW);
 *
 * RouteAuto drives it. Write routes in RED coordinates; RouteAuto spins them
 * for blue.
 *
 * Routes can't be changed once made: driveTo() etc. hand back a NEW, longer
 * route. So one route can safely be the start of another.
 *
 * Need the robot to do something new on the way (shoot, open a claw...)?
 * Add a value to Action, a method like collectTo() below, and handle it in
 * RouteAuto.
 */
public final class Route {

    public enum Action {
        DRIVE,    // just drive there
        COLLECT,  // run the collector when close to the point, stop on arrival
        LAUNCH,   // on arrival, launch into the hive. PLACEHOLDER until the
                  // launcher is built -- see RouteAuto.startLaunch()
    }

    public static final class Step {
        public final Pose target;
        public final Action action;

        Step(Pose target, Action action) {
            this.target = target;
            this.action = action;
        }
    }

    public final Pose start;
    public final List<Step> steps;

    private Route(Pose start, List<Step> steps) {
        this.start = start;
        this.steps = Collections.unmodifiableList(steps);
    }

    public static Route startAt(Pose start) {
        return new Route(start, new ArrayList<Step>());
    }

    public Route driveTo(Pose target) {
        return then(target, Action.DRIVE);
    }

    public Route collectTo(Pose target) {
        return then(target, Action.COLLECT);
    }

    public Route launchFrom(Pose target) {
        return then(target, Action.LAUNCH);
    }

    private Route then(Pose target, Action action) {
        List<Step> longer = new ArrayList<>(steps);
        longer.add(new Step(target, action));
        return new Route(start, longer);
    }
}
