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

    private final Turret turret = new Turret();
    private Follower follower;
    private TelemetryManager telemetryM;

    private boolean aiming = false;
    private boolean useHive = false;
    private Alliance alliance = Alliance.RED;
    private HiveGeometry.Tilt tilt = HiveGeometry.Tilt.AUDIENCE_UP;

    private boolean xPrev, aPrev, bPrev, yPrev, upPrev, downPrev;

    @Override
    public void runOpMode() {
        turret.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.refreshClass(TurretOdometryAimTest.class);
        PanelsConfigurables.refreshClass(Turret.class);
        PanelsConfigurables.refreshClass(TurretGeometry.class);

        telemetry.addLine("TURRET ODOMETRY AIM TEST");
        telemetry.addLine("X = home | A = aim on/off | B = reset pose | Y = mark/hive");
        telemetry.addLine("Robot must be on START_X/Y/HEADING when you press play.");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

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
        xPrev = gamepad1.x;
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;
        yPrev = gamepad1.y;
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
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
        if (useHive && !HiveGeometry.isMeasured()) {
            return "hive opening not measured yet: press Y to use the tape mark";
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
        telemetryM.addData("Target", useHive
                ? "hive " + alliance + " " + tilt
                : "tape mark");
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
            telemetryM.addData("Distance (in)", fmt(aim.distance));
            ShotTable shots = ShotTable.forRobot();
            telemetryM.addData("Table RPM", !shots.isConfigured() ? "table empty"
                    : fmt(shots.lookup(aim.distance))
                      + (shots.inRange(aim.distance) ? "" : " (OUTSIDE TABLE)"));
            telemetryM.addData("Aim error (deg)", fmt(aim.desiredDeg - turret.getAngle()));
        }
        telemetryM.addData("Turret angle (deg)", fmt(turret.getAngle()));
        telemetryM.addData("Turret state", turret.getState() + " - " + turret.getStatusNote());
        telemetryM.update(telemetry);
    }

    private static String fmt(double v) {
        return String.format("%.1f", v);
    }
}
