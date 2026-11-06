package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/*
 * Hands the robot's pose (and alliance) from auto to TeleOp.
 *
 * Auto saves where the robot is every loop. TeleOp, started a few seconds
 * later, picks that up as its starting pose -- so field-centric "forward" and
 * (later) odometry turret aiming are right without anyone lining the robot up.
 *
 * It lives in static fields, which survive from one op mode to the next but NOT
 * a redeploy or a power cycle (same as the turret's kept homing). That's fine:
 * neither happens between auto and TeleOp in a match.
 *
 * STALE POSES are the risk: run auto in practice, carry the robot somewhere,
 * start TeleOp -- the saved pose is now wrong. So TeleOp only defaults to it when
 * it's fresh (ageSeconds() under its limit) and shows it on the INIT screen, and
 * the driver can still pick "facing away" instead.
 */
public final class PoseHandoff {

    private PoseHandoff() {}

    private static Pose pose = null;
    private static Alliance alliance = null;
    private static long savedAtNanos = 0;

    /** Auto: call every loop (and in stop()). */
    public static void save(Pose robotPose, Alliance robotAlliance) {
        if (robotPose == null) return;
        pose = robotPose;
        alliance = robotAlliance;
        savedAtNanos = System.nanoTime();
    }

    public static boolean has() {
        return pose != null;
    }

    /** The last saved pose, or null if no auto has run since power-up. */
    public static Pose pose() {
        return pose;
    }

    /** The alliance auto ran as, or null. */
    public static Alliance alliance() {
        return alliance;
    }

    /** Seconds since auto last saved, or infinity if it never has. */
    public static double ageSeconds() {
        return has() ? (System.nanoTime() - savedAtNanos) / 1e9 : Double.POSITIVE_INFINITY;
    }

    /** Use once TeleOp has taken the pose, so a second TeleOp run can't reuse it. */
    public static void clear() {
        pose = null;
        alliance = null;
    }
}
