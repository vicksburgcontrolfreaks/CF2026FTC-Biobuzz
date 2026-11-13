package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.geometry.AimLead;
import org.firstinspires.ftc.teamcode.geometry.TagLocalizer;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.teamcode.subsystems.TurretDirector;

/**
 * The turret and camera measurements that DIFFER BETWEEN 5618 AND 6494.
 *
 * WHY THIS EXISTS
 * TurretGeometry and TagLocalizer each held ONE set of numbers, and the ones in
 * them were measured on 6494 (Shane, 2026-11-10 / 11-11). Deploy that to 5618
 * and it aims with the wrong robot's geometry without saying a word. Pedro's
 * drivetrain already has this problem solved in pedroPathing/Constants.java;
 * this is the same idea for the turret.
 *
 * HOW TO USE IT
 * Call install() ONCE at op-mode init, next to the PanelsConfigurables
 * refreshClass calls. It reads WhichRobot (the active config's name) and copies
 * that robot's numbers into the live fields on TurretGeometry / TagLocalizer /
 * TurretAim.
 *
 * WHY INSTALL ONCE AND NOT EVERY LOOP
 * Those live fields are @Configurable, so Panels can edit them on the field and
 * TurretGeometry.forRobot() picks the change up on the next loop. If install()
 * ran every loop it would stomp every edit -- the trap ShooterPivot's
 * loadRobotNumbers() warns about in its own comments. Running once at init
 * means: Panels edits stick for the rest of the op mode, and re-INIT puts the
 * stored numbers back, so you cannot strand yourself with a bad live edit.
 *
 * SO WHERE DO I PUT A NUMBER?
 *   - Trying a value out on the field  -> Panels, on TurretGeometry/TagLocalizer.
 *   - Keeping a value                  -> HERE, in this robot's block below.
 * These fields are deliberately NOT @Configurable: editing them in Panels after
 * install() has run would do nothing, which is worse than not offering it.
 *
 * NOTE ON THE TWO CAMERA_PITCH_DEG FIELDS
 * TagLocalizer and TurretAim each declare their own CAMERA_PITCH_DEG, and both
 * mean the same physical thing: how far the camera is tipped up from level.
 * They were separately set to 30.0 and could drift apart. install() sets BOTH
 * from CAMERA_PITCH_DEG here so there is one number to change. Collapsing them
 * into a single field is a worthwhile cleanup, but it touches TurretAim, which
 * DriveTeleOp uses in matches, so it is not bundled in with this split.
 */
public final class RobotProfile {

    private RobotProfile() {}

    /** Which robot install() last ran for, or "not installed". For telemetry. */
    private static String installedFor = "not installed";

    public static String installedFor() {
        return installedFor;
    }

    /**
     * Copy this robot's measurements into TurretGeometry, TagLocalizer and
     * TurretAim. Call once at init. Throws IllegalStateException (from
     * WhichRobot) if the active config name does not start with 5618 or 6494 --
     * better a refusal to INIT than aiming with the other robot's numbers.
     */
    public static void install() {
        switch (WhichRobot.get()) {
            case R5618:
                install5618();
                break;
            case R6494:
            default:
                install6494();
                break;
        }
        installedFor = WhichRobot.number();
    }

    // ---- 6494 ---------------------------------------------------------------
    private static void install6494() {
        // Turret axis relative to the POSE ORIGIN. Shane, tape measure,
        // 2026-11-10: 4.5 in. behind centre, laterally centred. Confirmed on the
        // robot 2026-11-11 by the spin test (the aim held through a spin instead
        // of wandering ~7 deg), so these two are measured AND proven.
        TurretGeometry.TURRET_OFFSET_X = -4.5;
        TurretGeometry.TURRET_OFFSET_Y = 0.0;
        TurretGeometry.TURRET_ZERO_DEG = 0.0;   // faces front, per the build team (Evan)
        TurretGeometry.TURRET_CCW_SIGN = 1.0;   // turret-bringup.md range test: ticks rise CCW

        // Camera on the turret, from the turret axis at FLOOR level, turret at 0.
        CAMERA_FORWARD = 4.5;    // MEASURED, Shane 2026-11-10
        CAMERA_LEFT = 0.0;       // NOT MEASURED -- assumed centred on the turret axis
        CAMERA_UP = 14.5;        // MEASURED (approx), Shane 2026-11-11
        CAMERA_PITCH_DEG = 35.0; // MEASURED (approx), Shane 2026-11-11. Was 30 by hand, 2026-10-22.
        CAMERA_YAW_DEG = 0.0;    // NOT MEASURED -- assumed square to the turret
        applyCamera();

        // TUNED ON THE ROBOT, Shane 2026-11-11. He stepped it with the D-pad in
        // Turret Odometry Aim Test and watched Aim error PEAK: 0.10 to 0.15 gave
        // the best results, and past 0.15 the aim visibly LEADS into turns. 0.12 is
        // the middle of that. Per robot because the right value is
        // 1 / (servo deg/sec * Turret.kP), and both differ between robots.
        AimLead.LEAD_SEC = 0.12;

        // THE SWITCH. false = camera-only aiming, which is what has been driven in
        // matches. Odometry aiming is only as good as the pose, and 6494's is due a
        // retune after the current mechanical changes -- so flip this to true once
        // the pose is trustworthy again, and check the Aim line on the Driver
        // Station says "odometry" rather than "camera". See TurretDirector.
        TurretDirector.USE_ODOMETRY_AIM = true;
    }

    // ---- 5618 ---------------------------------------------------------------
    private static void install5618() {
        // NOT MEASURED. NaN on purpose: TurretGeometry.isConfigured() is false
        // while either offset is NaN, so 5618 REFUSES to aim rather than aiming
        // with 6494's -4.5. Measure with a tape (turret axis relative to the
        // pose origin), prove it with the spin test in Turret Odometry Aim Test,
        // then write the numbers here.
        TurretGeometry.TURRET_OFFSET_X = Double.NaN;
        TurretGeometry.TURRET_OFFSET_Y = Double.NaN;
        // Same turret design on both robots, so these two carry over -- but
        // neither has been checked on 5618. The range test in TurretRangeTest
        // settles CCW_SIGN in a minute if the turret ever behaves backwards.
        TurretGeometry.TURRET_ZERO_DEG = 0.0;
        TurretGeometry.TURRET_CCW_SIGN = 1.0;

        // INHERITED, NOT MEASURED. These are the values 5618 was already running
        // before the split, so nothing it does today gets worse -- but every one
        // of them is 6494's mount or an outright guess. Camera aiming on 5618 is
        // only as good as these.
        CAMERA_FORWARD = 4.5;
        CAMERA_LEFT = 0.0;
        CAMERA_UP = 12.0;
        CAMERA_PITCH_DEG = 30.0;
        CAMERA_YAW_DEG = 0.0;
        applyCamera();

        // NOT TUNED on 5618. 0 is the lead switched off, which is the safe default --
        // a lead meant for another robot's servo and kP would aim ahead of the
        // target. Tune it the same way 6494's was: D-pad in Turret Odometry Aim
        // Test, watch Aim error PEAK, stop when it stops improving.
        AimLead.LEAD_SEC = 0.0;

        // Same switch. 5618 also needs TURRET_OFFSET_X/Y measured before this can do
        // anything -- TurretDirector falls back to the camera while the geometry is
        // NaN and says so, so turning it on early is harmless but pointless.
        TurretDirector.USE_ODOMETRY_AIM = false;
    }

    // ---- Collection: indexer motor direction --------------------------------
    /**
     * Which way this robot's indexer motor has to be set so that FORWARD means
     * "feed UP to the shooter".
     *
     * WHY THIS ONE IS A FUNCTION AND NOT SET BY install()
     * The turret and camera numbers above are state that install() pushes into
     * live @Configurable fields at op-mode init. The indexer direction can't
     * work that way: RobotHardware.applyCollectionDirections() is the only
     * place the motor direction is set, and it runs inside RobotHardware.init()
     * -- plus straight from Collection System Test, Indexer Load Test, Load
     * Sort Test and Load Recovery Test, NONE of which call install() at all.
     * install()-set state would leave those four running the other robot's
     * direction, in exactly the op modes you'd use to check the indexer.
     * Asking WhichRobot here means there is no init order to get wrong.
     *
     * HISTORY
     * 2026-11-11 (a65c48a) flipped this to REVERSE on BOTH robots, but the flip
     * was only ever for 5618's rebuilt collection system. 6494 was fine as
     * FORWARD and is put back here (Shane, 2026-11-12).
     *
     * Neither value is robot-tested since the split.
     */
    public static DcMotorSimple.Direction indexerDirection() {
        try {
            switch (WhichRobot.get()) {
                case R5618:
                    // The rebuilt collection system ran backwards as FORWARD.
                    // Flipped 2026-11-11; NOT robot-tested.
                    return DcMotorSimple.Direction.REVERSE;
                case R6494:
                default:
                    // Confirmed feeding UP on the robot, 2026-11-05.
                    return DcMotorSimple.Direction.FORWARD;
            }
        } catch (IllegalStateException e) {
            // Unknown robot: keep the default (6494's), same as Launcher does.
            return DcMotorSimple.Direction.FORWARD;
        }
    }

    // ---- plumbing -----------------------------------------------------------
    // Staged here by the install methods above, then pushed out by applyCamera()
    // so the two CAMERA_PITCH_DEG fields cannot be fed different numbers.
    private static double CAMERA_FORWARD, CAMERA_LEFT, CAMERA_UP, CAMERA_PITCH_DEG, CAMERA_YAW_DEG;

    private static void applyCamera() {
        TagLocalizer.CAMERA_FORWARD = CAMERA_FORWARD;
        TagLocalizer.CAMERA_LEFT = CAMERA_LEFT;
        TagLocalizer.CAMERA_UP = CAMERA_UP;
        TagLocalizer.CAMERA_PITCH_DEG = CAMERA_PITCH_DEG;
        TagLocalizer.CAMERA_YAW_DEG = CAMERA_YAW_DEG;

        // The duplicate. Same physical angle; see the class comment.
        TurretAim.CAMERA_PITCH_DEG = CAMERA_PITCH_DEG;
    }
}
