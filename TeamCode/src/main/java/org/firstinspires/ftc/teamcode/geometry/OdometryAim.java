package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

/*
 * OdometryAim: robot pose + target -> the turret angle to actually COMMAND.
 *
 * TurretGeometry.aimAngle() answers "which way is the target?" -- any angle in
 * (-180, 180]. The real turret can't go everywhere: it has hard stops, and
 * Turret.MIN_ANGLE / MAX_ANGLE are -140 / +140. That leaves an 80 deg DEAD ZONE
 * behind the turret's zero (centered on 180) that it can never point into.
 *
 *            0 (turret zero)
 *              |
 *    -140  .   |   .  +140       reachable: everything from -140 through 0 to +140
 *           \  |  /
 *            \ | /
 *        dead zone, 140..180..-140
 *
 * Because the travel is LESS than a full circle, every reachable direction has
 * exactly one turret angle. So no "which way round?" choice when the target is
 * reachable. The only decision is what to do when it ISN'T:
 *
 *   Park at whichever limit the target will come back out of. If the robot spins
 *   and the target drifts across the dead zone, the turret could swing all
 *   the way from +140 to -140 (280 deg) just as the target crosses 180, then
 *   back again on the next wobble. HYSTERESIS_DEG stops that: the turret only
 *   switches sides once the target is that far past the middle of the dead zone.
 *
 * No hardware here, so it is tested on a laptop (OdometryAimTest).
 */
public final class OdometryAim {

    private OdometryAim() {}

    /** Everything the op mode needs to command the turret and show telemetry. */
    public static final class Solution {
        /** Turret angle that points at the target, wrapped. May be unreachable. */
        public final double desiredDeg;
        /** What to send to Turret.setTargetAngle(). Always inside [minDeg, maxDeg]. */
        public final double commandDeg;
        /** True if commandDeg actually points at the target. */
        public final boolean reachable;
        /** How far outside the travel the target is, in degrees. 0 when reachable. */
        public final double outOfReachDeg;
        /** Flat distance from the turret axis to the target, inches. Feeds the RPM table later. */
        public final double distance;

        Solution(double desiredDeg, double commandDeg, boolean reachable,
                 double outOfReachDeg, double distance) {
            this.desiredDeg = desiredDeg;
            this.commandDeg = commandDeg;
            this.reachable = reachable;
            this.outOfReachDeg = outOfReachDeg;
            this.distance = distance;
        }
    }

    /**
     * @param currentDeg   where the turret is now (Turret.getAngle()). Only used
     *                     to pick a side when the target is out of reach.
     * @param minDeg       Turret.MIN_ANGLE
     * @param maxDeg       Turret.MAX_ANGLE
     * @param hysteresisDeg how far past the middle of the dead zone the target must
     *                     go before the turret switches to the other limit
     */
    public static Solution solve(TurretGeometry geometry, Pose robot,
                                 double targetX, double targetY, double currentDeg,
                                 double minDeg, double maxDeg, double hysteresisDeg) {
        double desired = geometry.aimAngle(robot, targetX, targetY);
        double distance = geometry.distanceTo(robot, targetX, targetY);
        double command = reachableCommand(desired, currentDeg, minDeg, maxDeg, hysteresisDeg);
        double miss = Math.abs(TurretGeometry.wrapDegrees(desired - command));
        boolean reachable = desired >= minDeg && desired <= maxDeg;
        return new Solution(desired, command, reachable, reachable ? 0.0 : miss, distance);
    }

    /**
     * The angle to command for a desired (wrapped) angle. Inside the travel: the
     * desired angle itself. In the dead zone: one of the two limits, chosen as
     * described at the top of this file.
     */
    public static double reachableCommand(double desiredDeg, double currentDeg,
                                          double minDeg, double maxDeg, double hysteresisDeg) {
        double desired = TurretGeometry.wrapDegrees(desiredDeg);
        if (desired >= minDeg && desired <= maxDeg) {
            return desired;
        }

        // Middle of the dead zone, measured going CCW from maxDeg toward minDeg + 360.
        // With limits of -140/+140 this is 180.
        double deadZoneWidth = (minDeg + 360.0) - maxDeg;
        double middle = maxDeg + deadZoneWidth / 2.0;

        // past > 0: the target is on the minDeg side of the middle. < 0: maxDeg side.
        double past = TurretGeometry.wrapDegrees(desired - middle);

        boolean onMaxSide = currentDeg >= (minDeg + maxDeg) / 2.0;
        if (onMaxSide) {
            return past > hysteresisDeg ? minDeg : maxDeg;
        }
        return past < -hysteresisDeg ? maxDeg : minDeg;
    }
}
