package org.firstinspires.ftc.teamcode.geometry;

import com.bylazar.configurables.annotations.Configurable;

/*
 * AimLead: aim where the target is GOING to be, not where it is.
 *
 * WHY A PLAIN P LOOP ALWAYS LAGS A MOVING TARGET
 *
 * Turret.closedLoopPower() is power = kP * error, and for a CR servo power is
 * roughly SPEED. So to keep moving at v deg/sec the loop has to be carrying a
 * standing error of v / (servoSpeed * kP) the whole time -- it only ever reacts
 * to error that has already happened. The faster the robot turns, the further
 * behind the turret sits, and it never catches up while the turning continues.
 *
 * Measured on 6494, 2026-11-11: tracking lag passed 40 deg during a spin.
 *
 * THE FIX
 *
 * The target's motion is PREDICTABLE. Most of it is just the robot rotating, and
 * the pose already knows that. So estimate how fast the desired turret angle is
 * changing and hand the turret a target that far AHEAD:
 *
 *     turret.setTargetAngle(commandDeg + LEAD_SEC * rate)
 *
 * The lead deliberately creates the error the P term needs, so the turret runs
 * at the right speed with the error centred near zero instead of trailing.
 *
 * WHAT IT CANNOT DO: beat the servo's top speed. Once kP * error passes
 * MAX_POWER the turret is already flat out and leading it further changes
 * nothing. Expect a real win at ordinary driving rates and nothing at all at a
 * full-speed spin. That ceiling is a MAX_POWER question, not this.
 *
 * WHY THE RATE IS DIFFERENCED RATHER THAN READ FROM THE POSE
 *
 * Differencing the DESIRED ANGLE catches everything at once -- the robot
 * rotating, the robot translating, and the target itself moving -- with no
 * dependence on which localizer is fitted or on Pedro internals. The worry with
 * differencing is noise, but the arithmetic is kind here: Pinpoint heading noise
 * of a few hundredths of a degree over a 20 ms loop is a couple of deg/sec of
 * rate noise, and a couple of deg/sec times LEAD_SEC is a small fraction of a
 * degree of lead. The smoothing below takes most of what is left.
 *
 * Plain numbers, no hardware: tested on a laptop (AimLeadTest).
 */
@Configurable
public class AimLead {

    /**
     * How far ahead to aim, in seconds of target motion. 0 DISABLES the lead
     * entirely, which is the default on purpose -- this is unproven on a robot,
     * and the whole point is to A/B it against 0 with Aim error PEAK.
     *
     * In theory the right value is 1 / (servoSpeedDegPerSec * Turret.kP), which
     * is the latency the P loop is effectively carrying. On 6494's 0.020 kP and
     * a servo somewhere near 200 deg/sec that is about 0.25 sec. Tune it on the
     * robot rather than trusting that: too little leaves lag, too much overshoots
     * ahead of the target and will look like the aim leading into turns.
     */
    public static double LEAD_SEC = 0.0;   // installed per robot by RobotProfile

    /** Low-pass on the rate estimate, per update. 1.0 = no smoothing at all. */
    public static double SMOOTHING = 0.30;

    /**
     * A one-step change in the desired angle bigger than this is not the target
     * moving -- it is the target crossing the turret's blind arc, which flips the
     * desired angle by about 180 deg in a single loop. Differencing that would
     * invent an enormous rate and fling the turret. Those steps are dropped.
     */
    public static double RATE_JUMP_DEG = 20.0;

    /** Sanity clamp on the estimate, deg/sec. Stops one bad sample dominating. */
    public static double MAX_RATE_DEG_PER_SEC = 400.0;

    private double lastDesiredDeg = Double.NaN;
    private double lastTimeSec = Double.NaN;
    private double rateDegPerSec = 0.0;
    private boolean droppedLastSample = false;

    /** Forget the history. Call when aiming is switched on, or the target changes. */
    public void reset() {
        lastDesiredDeg = Double.NaN;
        lastTimeSec = Double.NaN;
        rateDegPerSec = 0.0;
        droppedLastSample = false;
    }

    /**
     * Feed in the desired turret angle and the time, every loop.
     *
     * @param desiredDeg where the turret WANTS to point (OdometryAim's
     *                   desiredDeg, not commandDeg -- commandDeg is pinned at a
     *                   travel limit while the target is out of reach, so its
     *                   rate is zero exactly when the target is moving fastest)
     * @param timeSec    any steadily increasing clock, in seconds
     * @return the smoothed rate in deg/sec, same as rateDegPerSec()
     */
    public double update(double desiredDeg, double timeSec) {
        droppedLastSample = false;

        if (!Double.isNaN(lastDesiredDeg) && !Double.isNaN(lastTimeSec)) {
            double dt = timeSec - lastTimeSec;
            double delta = TurretGeometry.wrapDegrees(desiredDeg - lastDesiredDeg);

            if (dt <= 0 || Math.abs(delta) > RATE_JUMP_DEG) {
                // Same clock twice, or a blind-arc flip. Keep the old estimate
                // rather than inventing one; say so, so telemetry can show it.
                droppedLastSample = true;
            } else {
                double instant = delta / dt;
                if (instant > MAX_RATE_DEG_PER_SEC) instant = MAX_RATE_DEG_PER_SEC;
                if (instant < -MAX_RATE_DEG_PER_SEC) instant = -MAX_RATE_DEG_PER_SEC;
                rateDegPerSec += SMOOTHING * (instant - rateDegPerSec);
            }
        }

        lastDesiredDeg = desiredDeg;
        lastTimeSec = timeSec;
        return rateDegPerSec;
    }

    /** The smoothed rate of change of the desired angle, deg/sec. */
    public double rateDegPerSec() {
        return rateDegPerSec;
    }

    /** Degrees to add to the turret's target. 0 whenever LEAD_SEC is 0. */
    public double leadDeg() {
        return LEAD_SEC * rateDegPerSec;
    }

    /** True if the last update could not use its sample. For telemetry. */
    public boolean droppedLastSample() {
        return droppedLastSample;
    }
}
