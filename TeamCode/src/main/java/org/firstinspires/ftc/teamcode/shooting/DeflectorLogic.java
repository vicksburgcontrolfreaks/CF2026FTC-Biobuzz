package org.firstinspires.ftc.teamcode.shooting;

/*
 * DeflectorLogic: the deflector's math, with no hardware in it, so it's tested
 * on the laptop (DeflectorLogicTest). subsystems/Deflector reads the servo's
 * feedback and writes the power; everything it decides comes from here.
 *
 * THE MECHANISM (6494, 2026-11-06): the deflector sets the shot ARC. It's
 * driven by an Axon in continuous-rotation mode with a feedback wire, through
 * about 30:1 -- level to straight up was 2687 servo degrees, about 7.5 TURNS.
 * The feedback only knows where it is within ONE turn (0-360, then wraps), so:
 *
 *   1. Every loop, unwrap the reading into a running total (unwrapStep).
 *   2. Every match, someone holds it LEVEL and zeroes it (Deflector.zeroHere).
 *   3. Deflector angle = (total - total at zero) / SERVO_DEG_PER_DEFLECTOR_DEG.
 *
 * Unwrapping assumes the servo moves less than half a turn (180 servo deg) per
 * loop. At ~one turn per second that's fine down to ~400 ms loops; MAX_POWER
 * keeps a margin. A loop that stalls longer while it's moving loses a turn and
 * every angle after it is off by 12 deflector degrees -- re-zero if the arc
 * ever looks wrong.
 *
 * Angles: deflector degrees, 0 = level (parallel to the floor), 90 = straight up.
 */
public final class DeflectorLogic {

    private DeflectorLogic() {}

    /** 2687 servo deg level -> straight up, measured on 6494 (Shane, 2026-11-06). */
    public static double SERVO_DEG_PER_DEFLECTOR_DEG = 2687.0 / 90.0;

    /**
     * +1 = positive power raises the deflector. Servo Feedback Test (11/03):
     * + power = angle goes UP, and the 11/06 measurement went +2687 from level
     * to straight up. If it drives AWAY from its target, flip this.
     */
    public static double POWER_SIGN = 1.0;

    /** The arc positions, low to high (Shane, 2026-11-06). */
    public static double[] POSITIONS = {0, 60, 65, 70, 75, 80, 90};

    // Soft limits: never push past these. Just outside the positions so 0 and
    // 90 are reachable but a bad zero can't drive it far into the frame.
    public static double MIN_ANGLE = -2.0;
    public static double MAX_ANGLE = 92.0;

    // Position loop. Starting guesses -- tune on the robot.
    public static double kP = 0.08;          // power per deflector degree of error
    public static double MAX_POWER = 0.8;
    public static double MIN_POWER = 0.05;   // below this a CR servo just sits there
    public static double TOLERANCE_DEG = 0.5;

    /**
     * How far the raw 0-360 reading moved since last loop, in servo degrees,
     * assuming it moved less than half a turn. Crossing 360 -> 0 reads as a small
     * step forward, not a 360-degree jump back.
     */
    public static double unwrapStep(double previousRaw, double raw) {
        double change = raw - previousRaw;
        if (change > 180) change -= 360;
        if (change < -180) change += 360;
        return change;
    }

    /** Deflector degrees from the running servo total and the total at the zero. */
    public static double toDeflectorDeg(double servoTotal, double servoTotalAtZero) {
        return (servoTotal - servoTotalAtZero) / SERVO_DEG_PER_DEFLECTOR_DEG;
    }

    /** Servo power to move from angle toward target. 0 when close enough or at a limit. */
    public static double power(double angle, double target) {
        double error = target - angle;
        if (Math.abs(error) <= TOLERANCE_DEG) return 0;
        double p = Math.max(-MAX_POWER, Math.min(MAX_POWER, kP * error));
        if (Math.abs(p) < MIN_POWER) p = Math.copySign(MIN_POWER, p);
        if (angle >= MAX_ANGLE && p > 0) return 0;   // already too high: don't go higher
        if (angle <= MIN_ANGLE && p < 0) return 0;   // already too low: don't go lower
        return p * POWER_SIGN;
    }

    /** Index of the position closest to this angle. */
    public static int nearestIndex(double angle) {
        int best = 0;
        for (int i = 1; i < POSITIONS.length; i++) {
            if (Math.abs(POSITIONS[i] - angle) < Math.abs(POSITIONS[best] - angle)) best = i;
        }
        return best;
    }

    public static double clipToLimits(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }
}
