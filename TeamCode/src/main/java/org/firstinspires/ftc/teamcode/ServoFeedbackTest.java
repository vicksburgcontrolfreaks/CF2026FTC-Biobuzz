package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Servo Feedback Test — Axon MAX MK2 in CONTINUOUS ROTATION mode, with its
 * feedback wire on an analog port.
 *
 * In continuous mode we don't tell the servo WHERE to go, we tell it how fast
 * to spin (power -1.0 to 1.0, just like a motor). So the only way to know
 * where it is, is the feedback wire: it reads 0 V at 0 degrees up to about
 * 3.3 V at 360 degrees, then WRAPS back to 0 V and starts over.
 *
 * Config (Driver Station robot config):
 *   - Servo           -> servo port,  type "Continuous Rotation Servo", named SERVO_NAME below
 *   - Feedback wire   -> Expansion Hub analog port, type "Analog Input", named "pivotPos"
 *
 * Controls:
 *   Left stick up/down  = spin the servo (power is capped at MAX_POWER)
 *   Y                   = call where it is right now "0 degrees" (resets Total)
 *   A                   = run the direction test (short nudge each way)
 *   B                   = stop the direction test early
 */
@TeleOp(name = "Servo Feedback Test", group = "Test")
public class ServoFeedbackTest extends LinearOpMode {

    private static final String SERVO_NAME = "deflectorPivot";
    private static final String FEEDBACK_NAME = "pivotPos";

    // Max power from the stick. Kept low on purpose: a pivot has hard stops,
    // and a continuous servo will happily drive straight into them.
    private static final double MAX_POWER = 0.3;

    // Direction test: spin at TEST_POWER for TEST_SECONDS one way, then back.
    // If that moves the pivot too far, make TEST_SECONDS smaller.
    private static final double TEST_POWER = 0.15;
    private static final double TEST_SECONDS = 0.5;

    private CRServo servo;
    private AnalogInput feedback;

    // "Total" angle: keeps counting past 360 (or below 0) when the feedback wraps.
    private double lastAngle;
    private double totalDegrees = 0;

    private String testResult = "";

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        feedback = hardwareMap.get(AnalogInput.class, FEEDBACK_NAME);

        // Before START: nothing moves, but we can still read the wire.
        // Turn the pivot gently by hand and watch the angle change.
        lastAngle = readAngle();
        while (opModeInInit()) {
            updateTotal();
            telemetry.addLine("Nothing moves until START. Turn it by hand to check the wire.");
            showReadings(0);
            telemetry.update();
        }

        while (opModeIsActive()) {
            updateTotal();

            if (gamepad1.y) {
                totalDegrees = 0;
            }

            if (gamepad1.a) {
                runDirectionTest();
                continue;
            }

            double power = -gamepad1.left_stick_y * MAX_POWER;   // stick up is negative, so flip it
            if (Math.abs(power) < 0.02) power = 0;               // ignore stick drift
            servo.setPower(power);

            showReadings(power);
            telemetry.addLine();
            telemetry.addLine("Stick: spin   |   Y: set zero here   |   A: direction test");
            if (!testResult.isEmpty()) {
                telemetry.addLine();
                telemetry.addLine(testResult);
            }
            telemetry.update();
        }
        servo.setPower(0);
    }

    /** Feedback voltage turned into degrees, 0 to 360. */
    private double readAngle() {
        return feedback.getVoltage() / feedback.getMaxVoltage() * 360.0;
    }

    /**
     * Add up how far it moved since last time. The trick: if the angle jumps
     * by more than half a circle in one loop, it didn't really spin that fast —
     * it WRAPPED past 360 -> 0 (or 0 -> 360). So we fix the jump.
     */
    private void updateTotal() {
        double angle = readAngle();
        double change = angle - lastAngle;
        if (change > 180) change -= 360;    // wrapped going backwards (0 -> 360)
        if (change < -180) change += 360;   // wrapped going forwards (360 -> 0)
        totalDegrees += change;
        lastAngle = angle;
    }

    /** Nudge it + then -, and report which way + power actually turns it. */
    private void runDirectionTest() {
        double start = totalDegrees;
        if (!spinFor(TEST_POWER)) { testResult = "Direction test stopped early."; return; }
        double afterPlus = totalDegrees;
        if (!spinFor(-TEST_POWER)) { testResult = "Direction test stopped early."; return; }
        double afterMinus = totalDegrees;

        double plusMove = afterPlus - start;
        double minusMove = afterMinus - afterPlus;
        testResult = String.format(
                "DIRECTION TEST:%n  + power moved %+.1f deg%n  - power moved %+.1f deg%n  %s",
                plusMove, minusMove,
                Math.abs(plusMove) < 2 ? "Barely moved! Check wiring / try more power."
                        : (plusMove > 0 ? "+ power makes the angle go UP" : "+ power makes the angle go DOWN"));
    }

    /** Spin at a power for TEST_SECONDS. Returns false if B was pressed or the op mode stopped. */
    private boolean spinFor(double power) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TEST_SECONDS) {
            if (gamepad1.b) { servo.setPower(0); return false; }
            servo.setPower(power);
            updateTotal();
            telemetry.addLine("DIRECTION TEST running (B to stop)");
            showReadings(power);
            telemetry.update();
        }
        servo.setPower(0);
        // Let it coast to a stop before the next reading.
        ElapsedTime coast = new ElapsedTime();
        while (opModeIsActive() && coast.seconds() < 0.3) updateTotal();
        return opModeIsActive();
    }

    private void showReadings(double power) {
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Feedback voltage", "%.3f V", feedback.getVoltage());
        telemetry.addData("Angle (0-360)", "%.1f deg", lastAngle);
        telemetry.addData("Total (counts turns)", "%.1f deg", totalDegrees);
    }
}
