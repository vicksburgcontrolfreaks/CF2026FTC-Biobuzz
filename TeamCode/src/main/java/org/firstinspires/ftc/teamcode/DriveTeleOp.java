package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auton.AutoMenu;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.geometry.FieldCentric;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Match TeleOp. Gamepad 1 DRIVES, gamepad 2 runs everything else.
 *
 * FIELD-CENTRIC: stick forward = away from OUR drivers, whichever way the robot
 * faces. That needs the robot to know its heading, which comes from one of:
 *
 *   FROM AUTO    - auto saves its pose every loop (PoseHandoff); TeleOp starts
 *                  from it. Default when auto ran in the last AUTO_POSE_FRESH_SEC.
 *   FACING AWAY  - "the robot is pointed away from us right now." Use when no
 *                  auto ran (practice). Position is unknown (set to field
 *                  center) -- fine for driving, NOT for odometry turret aim.
 *
 * Either way, gamepad 1 BACK during the match = "the robot is facing away from
 * me right now": resets the heading (keeps x, y) and rumbles to confirm. Use it
 * if the controls ever feel rotated (a hard hit can knock the heading off).
 *
 * INIT menu (gamepad 1, same as Select Auto): D-pad up/down row, left/right
 * change, A lock, B unlock.
 */
@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_SPEED = 1.0; // 1.0 = full power. Lower (e.g. 0.5) only for new-driver practice.

    // A saved auto pose older than this isn't the default (it may be from a
    // practice run, and the robot has been carried since). It can still be
    // picked by hand. A match has ~8 s between auto and TeleOp.
    private static final double AUTO_POSE_FRESH_SEC = 60;

    private enum StartPose {
        FROM_AUTO("From auto"), FACING_AWAY("Facing away from us");
        private final String label;
        StartPose(String label) { this.label = label; }
        @Override public String toString() { return label; }
    }

    private final AutoMenu menu = new AutoMenu();
    private final AutoMenu.Choice<Alliance> alliance =
            menu.add("Alliance", Alliance.RED, Alliance.BLUE);
    private final AutoMenu.Choice<StartPose> startPose =
            menu.add("Start pose", StartPose.FACING_AWAY, StartPose.FROM_AUTO)
                    .allowWhen(p -> p != StartPose.FROM_AUTO || PoseHandoff.has());

    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        shooter = new ShootAprilTag(hardware);
        follower = Constants.createFollower(hardwareMap);

        // Default to auto's alliance and pose if auto just ran.
        if (PoseHandoff.has() && PoseHandoff.ageSeconds() < AUTO_POSE_FRESH_SEC) {
            if (PoseHandoff.alliance() != null) alliance.set(PoseHandoff.alliance());
            startPose.set(StartPose.FROM_AUTO);
        }
    }

    @Override
    public void init_loop() {
        menu.update(gamepad1);

        BuildInfo.show(telemetry);
        menu.show(telemetry);
        telemetry.addLine("");
        if (PoseHandoff.has()) {
            Pose p = PoseHandoff.pose();
            telemetry.addData("Auto pose", "(%.0f, %.0f) %.0f deg, %s, %.0f s ago%s",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()),
                    PoseHandoff.alliance(), PoseHandoff.ageSeconds(),
                    PoseHandoff.ageSeconds() < AUTO_POSE_FRESH_SEC ? "" : "  (OLD)");
            if (startPose.get() == StartPose.FROM_AUTO
                    && PoseHandoff.alliance() != null && PoseHandoff.alliance() != alliance.get()) {
                telemetry.addLine(">>> Auto ran as " + PoseHandoff.alliance()
                        + " but Alliance is " + alliance.get() + " -- check!");
            }
        } else {
            telemetry.addData("Auto pose", "none (no auto since power-up)");
        }
        if (startPose.get() == StartPose.FACING_AWAY) {
            telemetry.addLine("Point the robot AWAY from the " + alliance.get()
                    + " drivers before Start.");
        }
        telemetry.addLine("");
        telemetry.addLine("Gamepad1: sticks drive, BACK = 'robot faces away from me' heading reset");
        telemetry.addLine("Gamepad2: A collector (hold), Y align, B shoot  [old -- being rewritten]");
        telemetry.update();
    }

    @Override
    public void start() {
        Pose start;
        if (startPose.get() == StartPose.FROM_AUTO && PoseHandoff.has()) {
            start = PoseHandoff.pose();
        } else {
            start = new Pose(FieldConstants.CENTER, FieldConstants.CENTER,
                    alliance.get().awayFromDriversHeading());
        }
        follower.setStartingPose(start);
        // Taken. A second TeleOp run must not silently reuse a pose from before
        // this whole match of driving.
        PoseHandoff.clear();
    }

    @Override
    public void loop() {
        follower.update();   // localization only -- we drive the motors ourselves

        // ---- Gamepad 1: drive ----------------------------------------------
        if (gamepad1.backWasPressed()) {
            Pose p = follower.getPose();
            follower.setPose(new Pose(p.getX(), p.getY(), alliance.get().awayFromDriversHeading()));
            gamepad1.rumble(300);
        }

        double heading = follower.getPose().getHeading();
        double[] robot = FieldCentric.toRobot(
                -gamepad1.left_stick_y, gamepad1.left_stick_x,
                heading, alliance.get().awayFromDriversHeading());
        double forward = robot[0];
        double right = robot[1];
        double turn = gamepad1.right_stick_x;

        double den = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);
        hardware.lf.setPower((forward + right + turn) / den * DRIVE_SPEED);
        hardware.rf.setPower((forward - right - turn) / den * DRIVE_SPEED);
        hardware.lr.setPower((forward - right + turn) / den * DRIVE_SPEED);
        hardware.rr.setPower((forward + right - turn) / den * DRIVE_SPEED);

        // ---- Gamepad 2: OLD controls, unchanged until the rewrite ----------
        // (ShootAprilTag uses DECODE tag IDs 20/24 and blocks the loop.)
        int tagId = alliance.get() == Alliance.BLUE ? 20 : 24;
        if (gamepad2.a) {
            hardware.collector.setPower(0.5);
        } else {
            hardware.collector.setPower(0);
        }
        if (gamepad2.y) {
            shooter.alignRotationOnly(tagId);
        }
        if (gamepad2.b) {
            shooter.shoot(tagId);
        }

        Pose p = follower.getPose();
        telemetry.addData("Alliance", alliance.get());
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Away heading", "%.0f deg", Math.toDegrees(alliance.get().awayFromDriversHeading()));
        telemetry.update();
    }
}
