package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.geometry.AimLead;
import org.firstinspires.ftc.teamcode.geometry.TagLocalizer;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.shooting.ShotTable;
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
        // FLIPPED 2026-11-13. The turret now works the far side of its magnet, so
        // turret 0 points BACKWARD (away from the collector). Nothing was
        // unbolted: there are no hard stops, so which side of the magnet the
        // turret lives on is a software choice, and the magnet rides on the
        // turret under the camera so the camera-to-magnet angle never changed.
        //
        // WHY: the camera has to look back into the field while the robot sits
        // collector-first against a wall, which is the only workable convention
        // for the wall re-anchor (back-first points the camera at the wall).
        // Facing front with +/-90 of travel, that shot was unreachable.
        //
        // What it costs: the reachable arc is the robot's REAR half, so drivers
        // aim with their back to the hive and every shooting auto needs
        // re-checking. See docs/turret-odometry-aim-status.md.
        TurretGeometry.TURRET_ZERO_DEG = 180.0;
        TurretGeometry.TURRET_CCW_SIGN = 1.0;   // turret-bringup.md range test: ticks rise CCW

        // Camera on the turret, from the turret axis at FLOOR level, turret at 0.
        //
        // ALL OF THESE WERE RE-MEASURED 2026-11-14 (Shane) after the camera was
        // PHYSICALLY REMOUNTED on 6494. The previous mount sat 4.5 in ahead of
        // the turret axis in the direction the camera looks, which is where the
        // ball exits -- balls were striking the camera and its cord at large
        // turret angles (see docs/shot-map-2026-11-13.md). Every number below
        // belongs to the new mount; none of them is carried over.

        // NEGATIVE: the lens is now 1.25 in toward the COLLECTOR from the turret
        // axis, and turret-0 points AWAY from the collector since the flip, so
        // along the pointing direction the camera sits BEHIND the axis. The sign
        // is the whole content of this number -- a +1.25 here is a 2.5 in error
        // in every tag fix and every wall re-anchor, leaning one way.
        CAMERA_FORWARD = -1.25;  // MEASURED, Shane 2026-11-14
        CAMERA_LEFT = 0.0;       // NOT MEASURED -- assumed centred on the turret axis
        CAMERA_UP = 16.0;        // MEASURED to the lens, Shane 2026-11-14 (was 14.5)
        // 38 -> 35 for the new mount, MEASURED with a phone inclinometer against
        // the camera body, Shane 2026-11-14. (30 by hand 2026-10-22, 35 "approx"
        // 2026-11-11, 38 inclinometer 2026-11-13 on the OLD mount.)
        //
        // THE ROBOT MEASURES THIS BETTER THAN THE INCLINOMETER DOES, and that is
        // the check to run rather than re-reading the phone: Tag Pose Test solves
        // where the turret BASE is, the base is physically on the floor and level,
        // so its `lvl` reading IS the declared-vs-actual pitch error. It read
        // +0.8..+1.3 on the old mount at 38. If it comes back near +3 at 35, the
        // mount is really nearer 38 and the inclinometer is the wrong one.
        //
        // NOTE THIS ALSO MOVES CAMERA AIMING. applyCamera() pushes the same
        // number into TurretAim, whose bearingOf() rotates the camera frame down
        // to horizontal with it. More correct should mean better, but camera
        // aiming is what actually works in matches, so check DriveTeleOp aims
        // after deploying this.
        CAMERA_PITCH_DEG = 35.0;
        // CHECKED ON THE ROBOT 2026-11-14 and no longer a bare assumption. Shane
        // put a tag directly in front of the turret and read the bearing: the
        // first mount gave +8 deg, i.e. the camera body aimed 8 deg RIGHT of
        // where the turret points. He RE-SQUARED THE MOUNT rather than model it,
        // and a dead-ahead tag now reads within a degree or two of 0.
        //
        // Fixing it mechanically was the cheaper road by a long way: TurretAim --
        // the camera aiming that actually works in matches -- never reads this
        // field at all, so an 8 deg yaw would have biased every bearing it
        // produces AND leaked into TurretDirector's trim as fake pose drift.
        // AIM_BIAS_DEG would have aimed the turret right while making
        // isOnTarget() wrong by 8 deg in both directions, because DEADBAND_DEG is
        // tested BEFORE the bias is added (TurretAim line ~387). If this ever
        // comes back non-zero, teach TurretAim.bearingOf() the yaw at the source
        // the way WallFix.bearingDeg() already does -- do not reach for the bias.
        CAMERA_YAW_DEG = 0.0;
        applyCamera();

        // TUNED ON THE ROBOT, Shane 2026-11-11. He stepped it with the D-pad in
        // Turret Odometry Aim Test and watched Aim error PEAK: 0.10 to 0.15 gave
        // the best results, and past 0.15 the aim visibly LEADS into turns. 0.12 is
        // the middle of that. Per robot because the right value is
        // 1 / (servo deg/sec * Turret.kP), and both differ between robots.
        AimLead.LEAD_SEC = 0.12;

        // THE DEFAULT for the "Aim by" row in the INIT menu -- no longer the final
        // word, so it can be overridden at the robot without a redeploy.
        //
        // false = camera/tags, which is what has been driven in matches.
        //
        // Turned ON 2026-11-11 (e9b9bc0) and never driven. Turned back OFF
        // 2026-11-12: at Start the turret aimed from the pose, which in TeleOp and
        // Pollen Shot Tune FALLS BACK TO FIELD CENTRE when no auto handed one over.
        // Aiming at the hive from a pose the robot isn't at put the command outside
        // the +/-90 travel, so it clipped and sat pinned on a soft limit, ignoring
        // the tags (Shane, at the robot). Odometry aiming is only as good as the
        // pose, and 6494's drift is the open item -- see
        // docs/turret-odometry-aim-status.md.
        //
        // To try it again: pick "odometry (pose)" in the INIT menu rather than
        // editing this, and confirm the Turret telemetry line says odometry.
        TurretDirector.USE_ODOMETRY_AIM = false;

        // SHOT TABLE: EMPTY, not measured. Fill it from Shot Map's "spot" rows
        // (distance_in -> rpm_centre, turret near 0), nearest first. PER ROBOT,
        // and tied to THIS build: if 6494 is rebuilt and the flywheels move
        // relative to the turret (they sit ~0.5-1 in left of it now, Shane
        // 2026-11-13), re-run Shot Map -- the old table and offset fit are void.
        ShotTable.DISTANCES = new double[]{};
        ShotTable.RPMS = new double[]{};
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
        // FLIPPED 2026-11-14 for the 5618 rebuild (Jade), matching 6494's
        // 2026-11-13 flip: turret 0 points BACKWARD, away from the collector.
        // The magnet/limit numbers for this frame live in Turret.loadRobotNumbers().
        // Check after homing: does the camera point straight BACK? If not, this
        // (or the homing numbers) is off. CCW_SIGN carries over from 6494 --
        // Turret Aim Test settles it if the turret ever behaves backwards.
        TurretGeometry.TURRET_ZERO_DEG = 180.0;
        TurretGeometry.TURRET_CCW_SIGN = 1.0;

        // MEASURED on the rebuilt 5618, 2026-11-14 (Jade). Same frame as 6494:
        // from the turret axis at FLOOR level, turret at 0.
        // (Was inherited/guessed: forward 4.5, up 12, pitch 30.)
        CAMERA_FORWARD = -1.25;   // NEGATIVE: the camera sits 1.25 in BEHIND the turret axis
        CAMERA_LEFT = 0.0;        // centred left to right
        CAMERA_UP = 17.0;
        CAMERA_PITCH_DEG = 42.65;
        CAMERA_YAW_DEG = 0.0;
        applyCamera();

        // NOT TUNED on 5618. 0 is the lead switched off, which is the safe default --
        // a lead meant for another robot's servo and kP would aim ahead of the
        // target. Tune it the same way 6494's was: D-pad in Turret Odometry Aim
        // Test, watch Aim error PEAK, stop when it stops improving.
        AimLead.LEAD_SEC = 0.0;

        // Same default. 5618 also needs TURRET_OFFSET_X/Y measured before this can
        // do anything -- TurretDirector falls back to the camera while the geometry
        // is NaN and says so. The INIT menu won't even offer "odometry (pose)" on a
        // robot whose geometry is unmeasured, so on 5618 this row is camera-only
        // until the offsets are written above.
        TurretDirector.USE_ODOMETRY_AIM = false;

        // SHOT TABLE: EMPTY, not measured. 5618's turret sits centred over the
        // flywheels, so its offset fit should come out near zero -- run Shot Map
        // anyway and confirm it rather than assuming. Rows go here, not on 6494's.
        ShotTable.DISTANCES = new double[]{};
        ShotTable.RPMS = new double[]{};
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
                    // FORWARD again after the full 5618 rebuild (Jade, 2026-11-14:
                    // "the indexer does not need to be reversed"). It was REVERSE
                    // from 2026-11-11 for the earlier collection rebuild.
                    return DcMotorSimple.Direction.FORWARD;
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
