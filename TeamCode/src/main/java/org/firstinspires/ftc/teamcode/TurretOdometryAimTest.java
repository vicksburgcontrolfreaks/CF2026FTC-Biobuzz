package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.teamcode.geometry.OdometryAim;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Phase 2 of the turret plan: aim with ODOMETRY ONLY. No camera.
 *
 * Pinpoint says where the robot is, TurretGeometry turns that into a turret
 * angle, and the turret holds it while you drive around. If the geometry is
 * right, a laser pointer taped to the turret stays on the target the whole time.
 *
 * THE LASER TEST
 *   1. Put a tape mark on the field and type its spot into TARGET_X / TARGET_Y
 *      (Pedro inches). You don't need the hive measured for this.
 *   2. Put the robot on a tape-measured start spot; type it into START_*.
 *   3. INIT, then X to home, then A to start aiming.
 *   4. Drive slowly and spin in place. Watch the laser dot.
 *
 * What the dot tells you:
 *   - Stays on the mark everywhere           -> geometry is right. Phase 2 done.
 *   - Off by the same amount everywhere       -> TURRET_ZERO_DEG is off by that much.
 *   - Fine when still, drifts when SPINNING   -> TURRET_OFFSET_X/Y is wrong.
 *   - Fine at first, worse the longer you drive -> odometry drift, not the turret.
 *   - Turret turns the WRONG way as you spin  -> TURRET_CCW_SIGN.
 *
 * Controls (gamepad1):
 *   Sticks     - drive (robot-centric, scaled by DRIVE_SCALE)
 *   X          - home the turret
 *   A          - aiming on / off
 *   B          - "the robot is on the start spot again": reset pose to START_*
 *   Y          - target: tape mark <-> our hive's up cell
 *   D-pad up   - alliance RED / BLUE   (hive target only)
 *   D-pad down - hive tilt FAR_UP / AUDIENCE_UP (hive target only)
 */
@Configurable
// Shown again 2026-11-10 to measure TURRET_OFFSET_X/Y. Was hidden for competition
// on 2026-11-07; put @Disabled back above @TeleOp to hide it from the DS again.
@TeleOp(name = "Turret Odometry Aim Test", group = "Test")
public class TurretOdometryAimTest extends LinearOpMode {

    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING_DEG = 90;

    public static double TARGET_X = 72;
    public static double TARGET_Y = 132;

    // See OdometryAim: how far past the back of the dead zone the target must go
    // before the turret swings over to the other limit.
    public static double HYSTERESIS_DEG = 15;

    public static double DRIVE_SCALE = 0.5;

    // ---- The turret speed-up comparison, without Panels ---------------------
    // LEFT BUMPER swaps between the gains this robot loaded at init and the
    // BASELINE pair below, so the "did the faster turret help?" test can be run
    // from the gamepad. Aim error peak is the number to compare: during a steady
    // spin it IS the tracking lag. RIGHT BUMPER zeroes the peak.
    //
    // Baseline = what 6494 ran before the 2026-11-10 speed-up. Not hard-coded as
    // "the slow set" for both robots: the fast set is whatever Turret
    // loadRobotNumbers() gave THIS robot, captured at init.
    public static double BASELINE_KP = 0.010;
    public static double BASELINE_MAX_POWER = 0.55;

    // What tells a commanded SLEW apart from the turret LAGGING, so the peak can
    // ignore the first and measure the second.
    //
    // Judged by how fast the COMMAND moves, not by how big the error is. A magnitude
    // ceiling was tried first (45 deg) and it just clipped the answer: Shane read a
    // peak of "about 44" on 2026-11-11, which is the bound, not the lag. Real lag can
    // legitimately be large -- that is the whole point of measuring it.
    //
    // A steady spin moves the command a few degrees per loop; the limit switch when
    // the target crosses the blind arc moves it ~180 deg in ONE loop. Nothing in
    // between, so this threshold is not delicate.
    public static double COMMAND_JUMP_DEG = 20.0;

    private final Turret turret = new Turret();
    private Follower follower;
    private TelemetryManager telemetryM;

    private boolean aiming = false;
    private boolean useHive = false;
    private Alliance alliance = Alliance.RED;
    private HiveGeometry.Tilt tilt = HiveGeometry.Tilt.AUDIENCE_UP;

    private boolean xPrev, aPrev, bPrev, yPrev, upPrev, downPrev, lbPrev, rbPrev;

    private double loadedKp, loadedMaxPower;   // this robot's own gains, read at init
    private boolean baselineGains = false;
    private double peakAimError = 0;
    private double lastCommandDeg = Double.NaN;
    // True from a command jump until the turret has caught up again. The catch-up
    // is a slew, not lag, so none of it belongs in the peak.
    private boolean settlingAfterJump = false;
    private boolean wasReachable = false;

    @Override
    public void runOpMode() {
        RobotProfile.install();   // this robot's turret geometry, before any aiming
        turret.init(hardwareMap);
        // After init, so these are whatever loadRobotNumbers() gave THIS robot.
        loadedKp = Turret.kP;
        loadedMaxPower = Turret.MAX_POWER;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.refreshClass(TurretOdometryAimTest.class);
        PanelsConfigurables.refreshClass(Turret.class);
        PanelsConfigurables.refreshClass(TurretGeometry.class);
        PanelsConfigurables.refreshClass(HiveGeometry.class);   // type hive opening estimates live

        telemetry.addLine("TURRET ODOMETRY AIM TEST");
        telemetry.addLine("X = home | A = aim on/off | B = reset pose | Y = mark/hive");
        telemetry.addLine("LB = swap turret gains (speed-up test) | RB = reset the peak");
        telemetry.addLine("Robot must be on START_X/Y/HEADING when you press play.");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();
        // PRIME THE FOLLOWER BEFORE EVER CALLING setTeleOpDrive.
        // Pedro 2.0.2 keeps its OWN copy of the pose inside VectorCalculator, and
        // the only thing that fills it in is follower.update() --
        // startTeleopDrive() does not. setTeleOpDrive() then reads that copy's
        // heading unconditionally, so calling it first threw
        //   NullPointerException: Pose.getHeading() on a null object reference
        // at VectorCalculator.setTeleOpMovementVectors(VectorCalculator.java:328)
        // on the very first loop, the instant you pressed play (2026-11-10).
        // Every stock Pedro tuner in pedroPathing/Tuning.java does this same
        // startTeleopDrive() -> update() -> setTeleOpDrive() order; we skipped it.
        follower.update();

        while (opModeIsActive()) {
            handleButtons();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * DRIVE_SCALE,
                    -gamepad1.left_stick_x * DRIVE_SCALE,
                    -gamepad1.right_stick_x * DRIVE_SCALE,
                    true);
            follower.update();

            Pose robot = follower.getPose();
            TurretGeometry geometry = TurretGeometry.forRobot();
            double[] target = target();
            String blocked = whyNotAiming(geometry, target);

            OdometryAim.Solution aim = null;
            if (blocked == null) {
                aim = OdometryAim.solve(geometry, robot, target[0], target[1],
                        turret.getAngle(), Turret.MIN_ANGLE, Turret.MAX_ANGLE, HYSTERESIS_DEG);
                if (aiming) {
                    turret.setTargetAngle(aim.commandDeg);
                }
            }

            turret.update();
            showTelemetry(robot, geometry, target, aim, blocked);
        }

        turret.stop();
    }

    private void handleButtons() {
        if (gamepad1.x && !xPrev) {
            aiming = false;
            turret.startHoming();
        }
        if (gamepad1.a && !aPrev) {
            aiming = !aiming;
            // HOLD either way: aiming feeds it targets; off, it holds where it is.
            turret.setControlMode(Turret.ControlMode.HOLD);
        }
        if (gamepad1.b && !bPrev) {
            follower.setPose(startPose());
        }
        if (gamepad1.y && !yPrev) {
            useHive = !useHive;
        }
        if (gamepad1.dpad_up && !upPrev) {
            alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }
        if (gamepad1.dpad_down && !downPrev) {
            tilt = tilt.afterTip();
        }
        if (gamepad1.left_bumper && !lbPrev) {
            baselineGains = !baselineGains;
            Turret.kP = baselineGains ? BASELINE_KP : loadedKp;
            Turret.MAX_POWER = baselineGains ? BASELINE_MAX_POWER : loadedMaxPower;
            peakAimError = 0;   // each gain set gets its own peak
            settlingAfterJump = false;
            lastCommandDeg = Double.NaN;
            wasReachable = false;
        }
        if (gamepad1.right_bumper && !rbPrev) {
            peakAimError = 0;
        }
        xPrev = gamepad1.x;
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;
        yPrev = gamepad1.y;
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
        lbPrev = gamepad1.left_bumper;
        rbPrev = gamepad1.right_bumper;
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private double[] target() {
        if (useHive) {
            return HiveGeometry.aimPoint(alliance, tilt);
        }
        return new double[] { TARGET_X, TARGET_Y };
    }

    /** null if we can aim; otherwise the one thing to fix, in words. */
    private String whyNotAiming(TurretGeometry geometry, double[] target) {
        if (!geometry.isConfigured()) {
            return "TurretGeometry not configured: set TURRET_OFFSET_X/Y in Panels";
        }
        // isAimMeasured, not isMeasured: aiming reads x and y only, and insisting on
        // the opening HEIGHT blocked the hive target on a number aiming never uses.
        if (useHive && !HiveGeometry.isAimMeasured()) {
            return "hive opening x/y not set: press Y to use the tape mark";
        }
        if (Double.isNaN(target[0]) || Double.isNaN(target[1])) {
            return "target is NaN";
        }
        if (!turret.isHomed()) {
            return "turret not homed: press X";
        }
        return null;
    }

    private void showTelemetry(Pose robot, TurretGeometry geometry, double[] target,
                               OdometryAim.Solution aim, String blocked) {
        telemetryM.addData("Aiming", aiming ? "ON" : "off");
        // Which robot's numbers are loaded. If this does not say 6494 or 5618, the
        // geometry below is whatever was compiled in, not this robot's.
        telemetryM.addData("Profile", RobotProfile.installedFor());
        telemetryM.addData("Target", useHive
                ? "hive " + alliance + " " + tilt
                : "tape mark");
        if (useHive && !HiveGeometry.OPENING_MEASURED) {
            // So an aim that lands off the opening is not blamed on the turret.
            telemetryM.addLine("Hive opening is an ESTIMATE, not measured -- see HiveGeometry");
        }
        telemetryM.addData("Target x, y", fmt(target[0]) + ", " + fmt(target[1]));
        if (blocked != null) {
            telemetryM.addLine("CAN'T AIM: " + blocked);
        }
        telemetryM.addLine("");
        telemetryM.addData("Robot x, y", fmt(robot.getX()) + ", " + fmt(robot.getY()));
        telemetryM.addData("Robot heading (deg)", fmt(Math.toDegrees(robot.getHeading())));
        if (geometry.isConfigured()) {
            Pose t = geometry.turretPosition(robot);
            telemetryM.addData("Turret axis x, y", fmt(t.getX()) + ", " + fmt(t.getY()));
        }
        telemetryM.addLine("");
        if (aim != null) {
            telemetryM.addData("Wants (deg)", fmt(aim.desiredDeg));
            telemetryM.addData("Commanding (deg)", fmt(aim.commandDeg));
            telemetryM.addData("Reach", aim.reachable
                    ? "OK"
                    : "OUT OF REACH by " + fmt(aim.outOfReachDeg) + " deg - turn the robot");
            // The WHOLE miss, lag plus anything the travel limits will not let the
            // turret reach. Read this one to answer "is it pointing at the target";
            // read Aim error to answer "is the turret keeping up". They are the same
            // number only while Reach says OK. Showing just the total is what made a
            // parked turret report 9.6 deg of "error" on 2026-11-11 when nothing was
            // lagging -- the target was simply past the limit.
            telemetryM.addData("Total off target (deg)", fmt(aim.desiredDeg - turret.getAngle()));
            telemetryM.addData("Distance (in)", fmt(aim.distance));
            ShotTable shots = ShotTable.forRobot();
            telemetryM.addData("Table RPM", !shots.isConfigured() ? "table empty"
                    : fmt(shots.lookup(aim.distance))
                      + (shots.inRange(aim.distance) ? "" : " (OUTSIDE TABLE)"));
            // TRACKING error: commandDeg is what the turret was actually TOLD to do, so
            // this is purely how far behind it is. NOT desiredDeg -- that one "may be
            // unreachable" (see OdometryAim.Solution), and when the target swings into
            // the blind arc it runs right outside the turret's +/-90 travel. Using it
            // made the peak read 245 deg on 2026-11-11: the blind arc, not lag.
            double trackingError = aim.commandDeg - turret.getAngle();
            telemetryM.addData("Aim error (deg)", fmt(trackingError));
            // The peak is for comparing gains, so it only counts samples where that
            // comparison means something: aiming on, target reachable, and no big slew
            // in progress. A slew across the dead zone is a genuine 180 deg command
            // jump, not the turret falling behind, and one of those would swamp the
            // number for the rest of the run.
            // A big one-loop jump in the COMMAND is a slew starting -- the limit switch
            // across the blind arc moves it ~180 deg at once. Stop counting until the
            // turret has caught up, because the catch-up is the slew, not lag.
            if (!Double.isNaN(lastCommandDeg)
                    && Math.abs(aim.commandDeg - lastCommandDeg) > COMMAND_JUMP_DEG) {
                settlingAfterJump = true;
            }
            // The other discontinuity, and the one that was still leaking in: the target
            // coming BACK out of the blind arc. The turret is parked at a travel limit
            // and the target re-enters reach already sweeping at the robot's spin rate,
            // so the turret has to accelerate from a standstill to match something
            // already moving. Tens of degrees, every time, and it is a slew-rate story
            // rather than a gain one -- it would swamp a kP comparison. Shane hit this
            // on 2026-11-11 spinning all the way around at a moderate rate.
            if (aim.reachable && !wasReachable) {
                settlingAfterJump = true;
            }
            wasReachable = aim.reachable;
            if (settlingAfterJump && Math.abs(trackingError) <= Turret.TOLERANCE_DEG) {
                settlingAfterJump = false;   // caught up; back to measuring lag
            }
            lastCommandDeg = aim.commandDeg;

            boolean counts = aiming && aim.reachable && !settlingAfterJump;
            if (counts && Math.abs(trackingError) > peakAimError) {
                peakAimError = Math.abs(trackingError);
            }

            // Above this the turret is already at MAX_POWER, so kP changes nothing and
            // a gain comparison is really a MAX_POWER comparison. Worth saying on
            // screen: Shane's first run peaked around 44 deg with saturation at 40.
            double saturatesAtDeg = Turret.kP > 0 ? Turret.MAX_POWER / Turret.kP : 0;
            telemetryM.addData("Power-saturated past (deg)", fmt(saturatesAtDeg)
                    + (peakAimError > saturatesAtDeg ? "  <-- PEAK IS PAST THIS, kP is not the lever" : ""));
            // Say WHY it is not counting. A frozen peak with no explanation looks
            // exactly like a turret that has stopped falling behind.
            String why = counts ? ""
                    : !aiming ? "  [paused: aiming off]"
                    : !aim.reachable ? "  [paused: target out of reach]"
                    : "  [paused: catching up after a slew]";
            telemetryM.addData("Aim error PEAK (deg)", fmt(peakAimError) + why
                    + "  (LB swaps gains, RB resets)");
            telemetryM.addData("Gains", (baselineGains ? "BASELINE" : "robot's own")
                    + String.format("  kP %.3f  max %.2f", Turret.kP, Turret.MAX_POWER));
        }
        telemetryM.addData("Turret angle (deg)", fmt(turret.getAngle()));
        telemetryM.addData("Turret state", turret.getState() + " - " + turret.getStatusNote());
        telemetryM.update(telemetry);
    }

    private static String fmt(double v) {
        return String.format("%.1f", v);
    }
}
