package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name = "Shooter Velocity Test", group = "Test")
public class ShooterVelocityTest extends LinearOpMode {

    // REV HD Hex Motor: 28 ticks/rev at the encoder, 3:1 internal gearbox,
    // 1:1 (direct drive) from output shaft to wheel.
    private static final double TICKS_PER_REV = 28;
    private static final double MOTOR_GEAR_RATIO = 3.0;
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_REV * MOTOR_GEAR_RATIO;

    private static final double RPM_STEP = 100;
    private static final double MAX_RPM = 3000;

    // Starting spacing for the indexer servo (adjusts the gap between the two wheels
    // to accommodate pollen- vs. nectar-sized balls). Bumpers sweep this live so we
    // can find the real pollen/nectar values on the bench before hardcoding them.
    private static final double POLLEN_POSITION = 0.5;
    private static final double INDEXER_STEP_PER_LOOP = 0.005;

    // Live-tunable from FTC Dashboard (192.168.43.1:8080/dash) while this op mode
    // is running -- no redeploy needed. Seeded from the motor's current firmware values
    // in runOpMode() so the sliders start at the real baseline, not a guess.
    public static double P;
    public static double I;
    public static double D;
    public static double F;

    private DcMotorEx shooterLeft, shooterRight;
    private Servo indexer;
    private double targetRPM = 0.0;
    private double indexerPosition = POLLEN_POSITION;
    private boolean running = false;

    private boolean dpadUpPrev, dpadDownPrev, aPrev;

    @Override
    public void runOpMode() {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(indexerPosition);

        // Wheels face each other, so both run off the same commanded velocity
        // with one side physically reversed.
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients currentPIDF = shooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        P = currentPIDF.p;
        I = currentPIDF.i;
        D = currentPIDF.d;
        F = currentPIDF.f;

        // Mirrors telemetry to both the Driver Station and the FTC Dashboard web UI
        // (numeric values there are automatically graphable) from a single call site.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Dpad Up/Down: target RPM +/- 100");
        telemetry.addLine("A: toggle shooter on/off");
        telemetry.addLine("Bumpers: hold to sweep indexer spacing (find pollen/nectar positions)");
        telemetry.addLine("Tune P/I/D/F live at 192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean a = gamepad1.a;

            if (dpadUp && !dpadUpPrev) {
                targetRPM = Math.min(targetRPM + RPM_STEP, MAX_RPM);
            }
            if (dpadDown && !dpadDownPrev) {
                targetRPM = Math.max(targetRPM - RPM_STEP, 0.0);
            }
            if (a && !aPrev) {
                running = !running;
            }

            dpadUpPrev = dpadUp;
            dpadDownPrev = dpadDown;
            aPrev = a;

            if (gamepad1.right_bumper) {
                indexerPosition = Math.min(indexerPosition + INDEXER_STEP_PER_LOOP, 1.0);
            }
            if (gamepad1.left_bumper) {
                indexerPosition = Math.max(indexerPosition - INDEXER_STEP_PER_LOOP, 0.0);
            }
            indexer.setPosition(indexerPosition);

            // Re-applied every loop so edits made on the dashboard while running take
            // effect immediately.
            shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);
            shooterRight.setVelocityPIDFCoefficients(P, I, D, F);

            double commandedTicksPerSec = running ? rpmToTicksPerSec(targetRPM) : 0.0;
            shooterLeft.setVelocity(commandedTicksPerSec);
            shooterRight.setVelocity(commandedTicksPerSec);

            double leftRPM = ticksPerSecToRPM(shooterLeft.getVelocity());
            double rightRPM = ticksPerSecToRPM(shooterRight.getVelocity());

            telemetry.addData("Running", running);
            telemetry.addData("Target RPM", running ? targetRPM : 0.0);
            telemetry.addData("Left Actual RPM", leftRPM);
            telemetry.addData("Right Actual RPM", rightRPM);
            telemetry.addData("Indexer Position", indexerPosition);
            telemetry.update();
        }
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_WHEEL_REV / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_WHEEL_REV) * 60.0;
    }
}
