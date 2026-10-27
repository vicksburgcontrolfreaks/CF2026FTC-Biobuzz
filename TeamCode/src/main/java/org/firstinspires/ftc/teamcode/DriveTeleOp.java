package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    // ---- Driving knobs: change live in Panels (panels.bylazar.com), no redeploy ----
    // 1.0 = full power. 0.5 for practice.
    public static double DRIVE_SPEED = 0.5;
    // Robot-centric = "forward" is wherever the robot's front points. Simpler,
    // and doesn't depend on odometry heading. Field-centric = "forward" is always
    // away from the driver, however the robot is turned.
    public static boolean FIELD_CENTRIC = false;
    // If a stick drives the wrong way, flip ONLY the one that's wrong.
    public static boolean REVERSE_FORWARD = false;  // left stick up/down
    public static boolean REVERSE_STRAFE = false;   // left stick left/right
    public static boolean REVERSE_TURN = false;     // right stick

    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private int tagId = 20; // Default to blue alliance (ID 20)

    @Override
    public void init() {
        PanelsConfigurables.refreshClass(DriveTeleOp.class);
        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        shooter = new ShootAprilTag(hardware);
        try {
            follower = Constants.createFollower(hardwareMap);
            telemetry.addData("Status", "Follower initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Follower failed: " + e.getMessage());
        }

        // Alliance selection (Gamepad2)
        telemetry.addLine("Alliance Selection: Gamepad2 DPAD Up (Blue) or Down (Red)");
        if (gamepad2.dpad_up) {
            tagId = 20; // Blue alliance
            telemetry.addData("Alliance", "Blue (Tag ID 20)");
        } else if (gamepad2.dpad_down) {
            tagId = 24; // Red alliance
            telemetry.addData("Alliance", "Red (Tag ID 24)");
        }

        // Telemetry instructions
        telemetry.addLine("Gamepad1: Left Stick (Move/Strafe), Right Stick (Rotate), Y (Auto-Align)");
        telemetry.addLine("Gamepad2: A (Collector On/Off), Y (Auto-Align), B (Shoot)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y * (REVERSE_FORWARD ? -1 : 1);
        double x = gamepad1.left_stick_x * (REVERSE_STRAFE ? -1 : 1);
        double rx = gamepad1.right_stick_x * (REVERSE_TURN ? -1 : 1);

        // Field-centric: rotate the stick by MINUS the robot's heading. (This used
        // to rotate by +heading, which is right at 0 and 180 deg but flips the
        // sticks when the robot is turned sideways.) Heading 0 = however the robot
        // faced at INIT -- nothing sets a starting pose here.
        double heading = (FIELD_CENTRIC && follower != null) ? follower.getPose().getHeading() : 0.0;
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double scale = DRIVE_SPEED / den;

        hardware.lf.setPower((rotY + rotX + rx) * scale);
        hardware.rf.setPower((rotY - rotX - rx) * scale);
        hardware.lr.setPower((rotY - rotX + rx) * scale);
        hardware.rr.setPower((rotY + rotX - rx) * scale);

        // Collect: Gamepad2 A - run collector while held.
        // The three magazine beam breaks were removed 2026-10-27, so there is no
        // longer an automatic "stop when full" -- the driver decides.
        if (gamepad2.a) {
            hardware.collector.setPower(0.5);
        } else {
            hardware.collector.setPower(0);
        }

        // Auto-align rotation: Gamepad2 Y
        if (gamepad2.y) {
            shooter.alignRotationOnly(tagId);
        }

        // Shoot: Gamepad2 B
        if (gamepad2.b) {
            shooter.shoot(tagId);
        }

        // Update localizer
        if (follower != null) follower.update();

        telemetry.addData("Drive", (FIELD_CENTRIC ? "field-centric" : "robot-centric")
                + " at " + Math.round(DRIVE_SPEED * 100) + "%");
        telemetry.addData("Heading", follower != null ? Math.toDegrees(follower.getPose().getHeading()) : 0);
        telemetry.addData("Tag ID", tagId);
        telemetry.update();
    }
}