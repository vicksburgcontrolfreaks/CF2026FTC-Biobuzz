package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.geometry.TagLocalizer;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;

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
