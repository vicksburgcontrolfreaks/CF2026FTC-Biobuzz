package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {
    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private double collectorRPM = 0.0; // Manual collector RPM (0-2000)
    private double shooterRPM = 0.0; // Manual shooter RPM (0-3000)
    private boolean collectorOn = false; // Toggle for collector
    private boolean shooterOn = false; // Toggle for shooter
    private int tagId = 20; // Default to blue alliance (ID 20)
    private static final double TICKS_PER_REV = 28; // REV HD Hex Motor shaft
    private static final double COLLECTOR_GEAR_RATIO = 4.0; // Collector 20:1
    private static final double SHOOTER_GEAR_RATIO = 1.0; // Shooter no gearbox
    private Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false;
    private static final double DEBOUNCE_TIME = 0.2; // Seconds

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
        hardware.flipper.setPosition(0.0); // Reset servo
        debounceTimer.resetTimer();

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
        telemetry.addLine("Gamepad2: DPAD Up/Down (Collector RPM +/-), A (Collector On/Off)");
        telemetry.addLine("Gamepad2: Right/Left Bumper (Shooter RPM +/-), B (Shooter On/Off), X (Flipper)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Field-centric drive (Gamepad1)
        double y = -gamepad1.left_stick_y; // Forward/back
        double x = gamepad1.left_stick_x; // Strafe
        double rx = gamepad1.right_stick_x; // Rotate

        // Rotate for field-centric
        double heading = follower.getPose().getHeading();
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // Denominator for wheel powers
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Set powers
        hardware.lf.setPower((rotY + rotX + rx) / den);
        hardware.rf.setPower((rotY - rotX - rx) / den);
        hardware.lr.setPower((rotY - rotX + rx) / den);
        hardware.rr.setPower((rotY + rotX - rx) / den);

        // Auto-align rotation: Gamepad1 Y
        if (gamepad1.y) {
            shooter.alignRotationOnly(tagId);
        }

        // Manual controls (Gamepad2)
        // Collector: Dpad_up/down adjust speed (+/-100 RPM), A toggle on/off
        if (gamepad2.dpad_up) {
            collectorRPM += 1;
        } else if (gamepad2.dpad_down) {
            collectorRPM -= 1;
        }
        if (gamepad2.a && !lastA && debounceTimer.getElapsedTimeSeconds() > DEBOUNCE_TIME) {
            collectorOn = !collectorOn; // Toggle
            debounceTimer.resetTimer();
        }
        lastA = gamepad2.a;
        if (collectorOn) {
            double collectorTicksPerSec = collectorRPM * TICKS_PER_REV * COLLECTOR_GEAR_RATIO / 60.0;
            hardware.collector.setVelocity(collectorTicksPerSec);
        } else {
            hardware.collector.setPower(0.0);
        }

        // Shooter: Right bumper (+100 RPM), left bumper (-100 RPM), B toggle on/off
        if (gamepad2.right_bumper) {
            shooterRPM = Math.min(shooterRPM + 100, 3000);
        } else if (gamepad2.left_bumper) {
            shooterRPM = Math.max(shooterRPM - 100, 0);
        }
        if (gamepad2.b && !lastB && debounceTimer.getElapsedTimeSeconds() > DEBOUNCE_TIME) {
            shooterOn = !shooterOn; // Toggle
            debounceTimer.resetTimer();
        }
        lastB = gamepad2.b;
        if (shooterOn) {
            double shooterTicksPerSec = shooterRPM * TICKS_PER_REV * SHOOTER_GEAR_RATIO / 60.0;
            hardware.shooter.setVelocity(shooterTicksPerSec);
        } else {
            hardware.shooter.setPower(0.0);
        }

        // Flipper servo: X momentary (launch while held, reset when released)
        if (gamepad2.x) {
            hardware.flipper.setPosition(0.5); // Launch pos
        } else {
            hardware.flipper.setPosition(0.0); // Reset pos
        }

        // Update localizer
        follower.update();

        // Telemetry
        telemetry.addLine("Gamepad1: Left Stick (Move/Strafe), Right Stick (Rotate), Y (Auto-Align)");
        telemetry.addLine("Gamepad2: DPAD Up/Down (Collector RPM +/-), A (Collector On/Off)");
        telemetry.addLine("Gamepad2: Right/Left Bumper (Shooter RPM +/-), B (Shooter On/Off), X (Flipper)");
        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Collector RPM", collectorRPM + " (On: " + collectorOn + ")");
        telemetry.addData("Collector Actual RPM", (hardware.collector.getVelocity() / (TICKS_PER_REV * COLLECTOR_GEAR_RATIO)) * 60.0);
        telemetry.addData("Shooter RPM", shooterRPM + " (On: " + shooterOn + ")");
        telemetry.addData("Shooter Actual RPM", (hardware.shooter.getVelocity() / (TICKS_PER_REV * SHOOTER_GEAR_RATIO)) * 60.0);
        telemetry.addData("Servo Pos", hardware.flipper.getPosition());
        telemetry.update();
    }
}