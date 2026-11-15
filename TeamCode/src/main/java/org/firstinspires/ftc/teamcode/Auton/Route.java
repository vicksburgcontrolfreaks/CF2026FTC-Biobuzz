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
                  // (or after dwellSec on arrival -- see collectTo)
        LAUNCH,   // on arrival, aim and fire into the hive -- see RouteAuto.launchDone()
        LAUNCH_HERE, // LAUNCH without going anywhere: no path at all, fire from where
                     // the robot already is. Used to shoot the preloads at the START
                     // before driving. It is NOT a zero-length path -- those confuse
                     // the follower (which is why LAUNCH_OFF_WALL exists) -- the step
                     // simply has a null path and counts as arrived immediately.
        TIP_LAUNCH, // on arrival, wait until the camera sees our cell's tags (partner
                    // tipped the hive), then LAUNCH. Gives up / cuts firing short at
                    // RouteAuto.PARK_LEAVE_BY_SEC so we still make it to the park.
        TIP_LAUNCH_HERE, // TIP_LAUNCH without going anywhere (null path, like
                         // LAUNCH_HERE): wait for the tip where the robot stands.
                         // Far Wall Wait and Shoot, Shane 2026-11-14.
        YIELD,    // on arrival, hold still until the "yield until" time, to
                  // stay out of our alliance partner's way
    }

    public static final class Step {
        public final Pose target;
        public final Action action;
        /** Launch steps: seconds of firing. NaN = RouteAuto.LAUNCH_MAX_SEC. */
        public final double fireSec;
        /** Launch steps: stop at RouteAuto.PARK_LEAVE_BY_SEC so we still make the park. */
        public final boolean cutForPark;
        /**
         * Follower max power for THIS step, 0..1. NaN = the route's normal speed
         * (RouteAuto.MAX_POWER). Slow is for collecting, where the balls need time
         * to go in; it is not free -- see RouteAuto.startStep.
         */
        public final double speed;
        /**
         * Seconds to HOLD on arrival before the step finishes, with whatever the
         * step was doing still running. On a COLLECT step the collector keeps
         * turning for this long after the robot stops, which is how you sweep up
         * the balls that arrive at the wall with you. NaN = 0.
         */
        public final double dwellSec;
        /**
         * Seconds to GET THERE before we give up on this step and move on to the
         * next one. NaN = wait forever, which is what every step did before
         * 2026-11-14.
         *
         * WHY IT EXISTS: a step only ends when the follower says it arrived (see
         * RouteAuto.autonomousPathUpdate), and the follower is told to hold
         * position at the end of its path. A robot wedged on a ball never
         * arrives, so it pushes against it for the rest of the match and every
         * later step -- including the park -- never runs. That is exactly what
         * Garden Shoot 2X did on 5618 (Shane, 2026-11-14): it stalled a few
         * inches short of the garden corner and sat there.
         *
         * It times the DRIVING only. On a LAUNCH step the clock stops once the
         * robot arrives -- fireSec and cutForPark own the firing -- and a
         * LAUNCH_HERE step has no driving to time at all.
         *
         * Set it with Route.timeout(). It is a BACKSTOP, not a schedule: make it
         * comfortably longer than the leg really takes (read "Step time" off the
         * Driver Station) or it will cut good runs short.
         */
        public final double timeoutSec;

        Step(Pose target, Action action, double fireSec, boolean cutForPark,
             double speed, double dwellSec, double timeoutSec) {
            this.target = target;
            this.action = action;
            this.fireSec = fireSec;
            this.cutForPark = cutForPark;
            this.speed = speed;
            this.dwellSec = dwellSec;
            this.timeoutSec = timeoutSec;
        }
    }

    public final Pose start;
    public final List<Step> steps;
    /**
     * Index of the first PARK step, or -1 if the route never said. Set by
     * beginPark().
     *
     * RouteAuto treats it as a deadline: at PARK_LEAVE_BY_SEC, whatever the
     * robot is doing, it abandons it and drives the park steps from here (see
     * RouteAuto.jumpTo). Scoring the park is worth more than finishing the
     * route, and a route that gets stuck scores neither.
     */
    public final int parkFrom;

    private Route(Pose start, List<Step> steps, int parkFrom) {
        this.start = start;
        this.steps = Collections.unmodifiableList(steps);
        this.parkFrom = parkFrom;
    }

    public static Route startAt(Pose start) {
        return new Route(start, new ArrayList<Step>(), -1);
    }

    /**
     * "Everything after this is the park." Call it once, right before the
     * driveTo() steps that go to the park spot. It adds no step of its own.
     *
     * That is what lets RouteAuto bail out to the park when the route runs long
     * or gets stuck, instead of running out of time partway through the scoring
     * steps. A route that doesn't call it behaves exactly as it always has:
     * every step runs in order, and a stuck step stops the route.
     *
     * Mark the WHOLE park, not just the last point. Garden Shoot 2X needs three
     * waypoints to get around the hive and the red-wall flower, and jumping
     * straight to the last one would drive the diagonal that hit the hive on
     * 2026-11-14.
     */
    public Route beginPark() {
        if (parkFrom >= 0) return this;   // already marked; the first call wins
        return new Route(start, new ArrayList<>(steps), steps.size());
    }

    /**
     * Give the step just added a time limit for GETTING THERE, in seconds. See
     * Step.timeoutSec for what it does and does not cover.
     *
     *     .collectTo(Waypoints.GARDEN_CORNER, 0.2, 2).timeout(8)
     */
    public Route timeout(double seconds) {
        if (steps.isEmpty()) {
            throw new IllegalStateException("timeout() needs a step before it");
        }
        List<Step> copy = new ArrayList<>(steps);
        Step last = copy.remove(copy.size() - 1);
        copy.add(new Step(last.target, last.action, last.fireSec, last.cutForPark,
                last.speed, last.dwellSec, seconds));
        return new Route(start, copy, parkFrom);
    }

    public Route driveTo(Pose target) {
        return then(target, Action.DRIVE);
    }

    public Route collectTo(Pose target) {
        return then(target, Action.COLLECT);
    }

    /**
     * Collect to a point at your own speed, then keep the collector running for
     * dwellSec after the robot stops.
     *
     * @param speed    follower max power for this leg, 0..1
     * @param dwellSec seconds to sit still with the collector still turning
     */
    public Route collectTo(Pose target, double speed, double dwellSec) {
        return then(new Step(target, Action.COLLECT, Double.NaN, false, speed, dwellSec, Double.NaN));
    }

    /** Drive to a point at your own speed instead of the route's normal speed. */
    public Route driveTo(Pose target, double speed) {
        return then(new Step(target, Action.DRIVE, Double.NaN, false, speed, Double.NaN, Double.NaN));
    }

    /**
     * Fire from wherever the robot already is, without driving anywhere. Use it
     * to shoot the preloads at the START, and to shoot again after a driveTo()
     * has put the robot back on a spot it has already been -- `driveTo(X)` then
     * `launchHere()` is the way to express "stop there and shoot", because
     * `launchFrom(X)` right after `driveTo(X)` would build a ZERO-LENGTH path.
     *
     * The step has no path (see Action.LAUNCH_HERE) so it needs no target.
     * cutForPark is on: it costs nothing on a first step (RouteAuto only cuts a
     * launch that has another step after it) and protects the park on a later one.
     */
    public Route launchHere(double fireSec) {
        return then(new Step(null, Action.LAUNCH_HERE, fireSec, true, Double.NaN, Double.NaN, Double.NaN));
    }

    public Route launchFrom(Pose target) {
        return then(target, Action.LAUNCH);
    }

    /**
     * Launch for a set time instead of LAUNCH_MAX_SEC, and stop at
     * PARK_LEAVE_BY_SEC if a long Wait would make us miss the park.
     */
    public Route launchFrom(Pose target, double fireSec) {
        return then(new Step(target, Action.LAUNCH, fireSec, true, Double.NaN, Double.NaN, Double.NaN));
    }

    public Route tipLaunchFrom(Pose target) {
        return then(new Step(target, Action.TIP_LAUNCH, Double.NaN, true, Double.NaN, Double.NaN, Double.NaN));
    }

    /**
     * Wait for the tip WHERE WE STAND, then aim RouteAuto.TIP_AIM_SEC and fire
     * fireSec. No path. Cut short at PARK_LEAVE_BY_SEC like every tip launch.
     */
    public Route tipLaunchHere(double fireSec) {
        return then(new Step(null, Action.TIP_LAUNCH_HERE, fireSec, true, Double.NaN, Double.NaN, Double.NaN));
    }

    // ("yield" by itself is a Java keyword, so this is yieldAt.)
    public Route yieldAt(Pose target) {
        return then(target, Action.YIELD);
    }

    // Does any step do this? (e.g. the menu only shows "Yield until" for
    // routes that have a YIELD step.)
    public boolean has(Action action) {
        for (Step step : steps) {
            if (step.action == action) return true;
        }
        return false;
    }

    private Route then(Pose target, Action action) {
        return then(new Step(target, action, Double.NaN, false, Double.NaN, Double.NaN, Double.NaN));
    }

    private Route then(Step step) {
        List<Step> longer = new ArrayList<>(steps);
        longer.add(step);
        return new Route(start, longer, parkFrom);
    }
}
