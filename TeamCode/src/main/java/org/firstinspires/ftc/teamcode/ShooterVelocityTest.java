package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Shooter Velocity Test", group = "Test")
public class ShooterVelocityTest extends LinearOpMode {

    // REV HD Hex Motor: 28 ticks/rev at the encoder, 3:1 internal gearbox,
    // 1:1 (direct drive) from output shaft to wheel.
    private static final double TICKS_PER_REV = 28;
    private static final double MOTOR_GEAR_RATIO = 3.0;
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_REV * MOTOR_GEAR_RATIO;

    private static final double RPM_STEP = 100;
    private static final double MAX_RPM = 3000;

    // Indexer servo positions (adjusts the gap between the two wheels to accommodate
    // pollen- vs. nectar-sized balls), found by bench-sweeping on 2026-10-01.
    private static final double POLLEN_POSITION = 0.48;
    private static final double NECTAR_POSITION = 0.135;

    // Live-tunable from the Panels dashboard (panels.bylazar.com) while this op mode
    // is running -- no redeploy needed. Seeded from the motor's current firmware values
    // in runOpMode() so the sliders start at the real baseline, not a guess.
    public static double P;
    public static double I;
    public static double D;
    public static double F;

    private DcMotorEx shooterLeft, shooterRight;
    private Servo indexer;
    private double targetRPM = 0.0;
    private boolean onNectar = false;
    private boolean running = false;

    private boolean dpadUpPrev, dpadDownPrev, aPrev, xPrev;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(POLLEN_POSITION);

        // Wheels face each other, so both run off the same commanded velocity
        // with one side physically reversed.
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients currentPIDF = shooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        P = currentPIDF.p;
        I = currentPIDF.i;
        D = currentPIDF.d;
        F = currentPIDF.f;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetry.addLine("Dpad Up/Down: target RPM +/- 100");
        telemetry.addLine("A: toggle shooter on/off");
        telemetry.addLine("X: toggle indexer between pollen/nectar spacing");
        telemetry.addLine("Tune P/I/D/F live at panels.bylazar.com");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean a = gamepad1.a;
            boolean x = gamepad1.x;

            if (dpadUp && !dpadUpPrev) {
                targetRPM = Math.min(targetRPM + RPM_STEP, MAX_RPM);
            }
            if (dpadDown && !dpadDownPrev) {
                targetRPM = Math.max(targetRPM - RPM_STEP, 0.0);
            }
            if (a && !aPrev) {
                running = !running;
            }
            if (x && !xPrev) {
                onNectar = !onNectar;
            }

            dpadUpPrev = dpadUp;
            dpadDownPrev = dpadDown;
            aPrev = a;
            xPrev = x;

            indexer.setPosition(onNectar ? NECTAR_POSITION : POLLEN_POSITION);

            // Re-applied every loop so edits made on the dashboard while running take
            // effect immediately.
            shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);
            shooterRight.setVelocityPIDFCoefficients(P, I, D, F);

            double commandedTicksPerSec = running ? rpmToTicksPerSec(targetRPM) : 0.0;
            shooterLeft.setVelocity(commandedTicksPerSec);
            shooterRight.setVelocity(commandedTicksPerSec);

            double leftRPM = ticksPerSecToRPM(shooterLeft.getVelocity());
            double rightRPM = ticksPerSecToRPM(shooterRight.getVelocity());

            telemetryM.addData("Running", running);
            telemetryM.addData("Target RPM", running ? targetRPM : 0.0);
            telemetryM.addData("Left Actual RPM", leftRPM);
            telemetryM.addData("Right Actual RPM", rightRPM);
            telemetryM.addData("Indexer Mode", onNectar ? "Nectar" : "Pollen");
            telemetryM.update(telemetry);
        }
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_WHEEL_REV / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_WHEEL_REV) * 60.0;
    }
}
