package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_SPEED = 0.5; // 1.0 = full power; 0.5 for practice
    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private int tagId = 20; // Default to blue alliance (ID 20)

    @Override
    public void init() {
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
        // Field-centric drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double heading = follower.getPose().getHeading();
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        hardware.lf.setPower((rotY + rotX + rx) / den * DRIVE_SPEED);
        hardware.rf.setPower((rotY - rotX - rx) / den * DRIVE_SPEED);
        hardware.lr.setPower((rotY - rotX + rx) / den * DRIVE_SPEED);
        hardware.rr.setPower((rotY + rotX - rx) / den * DRIVE_SPEED);

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
        follower.update();

        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.addData("Tag ID", tagId);
        telemetry.update();
    }
}