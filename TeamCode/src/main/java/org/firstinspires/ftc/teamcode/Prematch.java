package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Prematch — the deflector servo (Axon MAX MK2, CONTINUOUS ROTATION) and its
 * feedback wire on an analog port.
 *
 * In continuous mode we don't tell the servo WHERE to go, we tell it how fast
 * to spin (power -1.0 to 1.0, just like a motor). So the only way to know
 * where it is, is the feedback wire: it reads 0 V at 0 degrees up to about
 * 3.3 V at 360 degrees, then WRAPS back to 0 V and starts over.
 *
 * Config (Driver Station robot config):
 *   - Servo           -> servo port,  type "Continuous Rotation Servo", named SERVO_NAME below
 *   - Feedback wire   -> Control Hub analog 1, type "Analog Input", named "deflectorPos"
 *                        (same port on both 5618 and 6494, confirmed 2026-11-06)
 *
 * Controls:
 *   Right trigger       = move it RIGHT  (squeeze harder = faster, capped at MAX_POWER)
 *   Left trigger        = move it LEFT
 *   Y                   = call where it is right now "0 degrees" (resets Total)
 *   A                   = run the direction test (short nudge each way)
 *   B                   = stop the direction test early
 *   X                   = clear the min/max voltage capture
 *   D-pad up            = drive to TARGET_DEGREES and stop (B cancels)
 */
// HIDDEN for the 2026-11-08 competition (Shane): despite the name this is a
// deflector feedback test, and its zero (Y) is NOT the one TeleOp/auto use --
// the drive team must run Deflector Zero. Delete this line to bring it back.
@Disabled
@TeleOp(name = "Prematch", group = "Utility")
public class Prematch extends LinearOpMode {

    private static final String SERVO_NAME = "deflector";
    private static final String FEEDBACK_NAME = "deflectorPos";

    // Max power from the stick. Kept low on purpose: a pivot has hard stops,
    // and a continuous servo will happily drive straight into them.
    private static final double MAX_POWER = 0.3;

    // Direction test: spin at TEST_POWER for TEST_SECONDS one way, then back.
    // If that moves the pivot too far, make TEST_SECONDS smaller.
    private static final double TEST_POWER = 0.15;
    private static final double TEST_SECONDS = 0.5;

    // ---- Go to a target ----------------------------------------------------
    // Where we want the deflector to end up, measured as TOTAL degrees from
    // wherever Y last called zero. 1700 is about 4.7 full turns of the servo
    // output shaft, so this is deliberately the wrap-counting "Total" reading
    // and NOT the 0-360 "Angle now" reading. A placeholder for today: the real
    // number comes out of range testing, and eventually a departure angle
    // picked from the distance to the target.
    private static final double TARGET_DEGREES = 1700;

    // Which sign of power makes the angle reading go UP. The direction test (A)
    // prints this for you -- run it once and set this to match, or the go-to
    // will drive confidently away from the target until it times out.
    private static final boolean POSITIVE_POWER_INCREASES_ANGLE = true;

    // Close enough to stop. A coasting CR servo will never land exactly.
    private static final double GOTO_TOLERANCE_DEG = 5;

    // Inside this much error, back off to GOTO_SLOW_POWER so the servo is not
    // still at full speed when it arrives and sails straight past.
    private static final double GOTO_APPROACH_DEG = 120;
    private static final double GOTO_SLOW_POWER = 0.10;

    // Hard stop on the whole move. Without this, anything that blocks the
    // deflector means the servo grinds against it until someone pulls the
    // battery.
    private static final double GOTO_TIMEOUT_S = 15;

    // Which way is "right". The code cannot work this out on its own -- whether
    // + power swings the deflector right or left depends on how the servo is
    // mounted and geared. Run the direction test (A), or just tap the right
    // trigger: if the deflector goes LEFT, change this to false and redeploy.
    private static final boolean RIGHT_TRIGGER_IS_POSITIVE_POWER = true;

    // Below this, a trigger counts as not pressed. Triggers often rest a hair
    // off zero, and without this the servo would creep on its own.
    private static final double TRIGGER_DEADBAND = 0.05;

    // How many degrees the feedback wire covers from 0 V to its max voltage.
    // THIS IS AN ASSUMPTION, not a measurement. 360 is the usual spec for an
    // Axon MAX MK2, but nobody has checked it on the deflector yet, so treat
    // the Angle readings below as provisional until you have. Measure it with
    // the min/max capture: turn the deflector hard stop to hard stop, note the
    // voltage span, then compare against how far it physically travelled.
    private static final double ASSUMED_SPAN_DEG = 360.0;

    private CRServo servo;
    private AnalogInput feedback;

    // "Total" angle: keeps counting past 360 (or below 0) when the feedback wraps.
    private double lastAngle;
    private double totalDegrees = 0;

    // Lowest and highest voltage seen since the op mode started (or since X).
    // Turning the deflector by hand between its hard stops fills these in, which
    // is how you find its real range without having to stare and remember.
    private double minVolts = Double.MAX_VALUE;
    private double maxVolts = -Double.MAX_VALUE;

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
            if (gamepad1.x) { resetRange(); }
            telemetry.addLine("Nothing moves until START. Turn it by hand to check the wire.");
            telemetry.addLine("Turn it hard stop to hard stop to capture the range.");
            showReadings(0);
            telemetry.addLine();
            telemetry.addLine("X: clear the min/max capture");
            telemetry.update();
        }

        while (opModeIsActive()) {
            updateTotal();

            if (gamepad1.y) {
                totalDegrees = 0;
            }

            if (gamepad1.x) {
                resetRange();
            }

            if (gamepad1.a) {
                runDirectionTest();
                continue;
            }

            if (gamepad1.dpad_up) {
                runToTotal(TARGET_DEGREES);
                continue;
            }

            // Right trigger goes one way, left trigger the other. Subtracting
            // them means pressing both at once cancels out instead of fighting,
            // and how FAR you squeeze sets the speed.
            double command = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(command) < TRIGGER_DEADBAND) {
                command = 0;
            }
            if (!RIGHT_TRIGGER_IS_POSITIVE_POWER) {
                command = -command;
            }
            double power = command * MAX_POWER;
            servo.setPower(power);

            showReadings(power);
            telemetry.addData("Triggers", "L %.2f   R %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addLine();
            telemetry.addLine("Right trigger: move RIGHT   |   Left trigger: move LEFT");
            telemetry.addLine("Y: set zero here   |   A: direction test   |   X: clear min/max");
            telemetry.addLine(String.format("D-pad up: go to %.0f deg   |   B: stop a move",
                    TARGET_DEGREES));
            if (!testResult.isEmpty()) {
                telemetry.addLine();
                telemetry.addLine(testResult);
            }
            telemetry.update();
        }
        servo.setPower(0);
    }

    /**
     * Feedback voltage turned into degrees, using ASSUMED_SPAN_DEG. Read that
     * comment before you trust this number -- the voltage is measured, the
     * degrees are scaled from an assumption.
     */
    private double readAngle() {
        return feedback.getVoltage() / feedback.getMaxVoltage() * ASSUMED_SPAN_DEG;
    }

    /** Forget the captured range and start the min/max over. */
    private void resetRange() {
        minVolts = Double.MAX_VALUE;
        maxVolts = -Double.MAX_VALUE;
    }

    /**
     * Add up how far it moved since last time. The trick: if the angle jumps
     * by more than half a circle in one loop, it didn't really spin that fast —
     * it WRAPPED past 360 -> 0 (or 0 -> 360). So we fix the jump.
     */
    private void updateTotal() {
        double volts = feedback.getVoltage();
        if (volts < minVolts) { minVolts = volts; }
        if (volts > maxVolts) { maxVolts = volts; }

        double angle = readAngle();
        double change = angle - lastAngle;
        if (change > 180) change -= 360;    // wrapped going backwards (0 -> 360)
        if (change < -180) change += 360;   // wrapped going forwards (360 -> 0)
        totalDegrees += change;
        lastAngle = angle;
    }

    /**
     * Drive until Total reaches target, then stop. This is the whole idea of a
     * continuous servo with a feedback wire: the servo cannot be told to go to
     * an angle, so we watch the wire and decide when to stop ourselves.
     *
     * Three things keep it from misbehaving:
     *   - it slows down near the target, because a servo at full power coasts
     *     past the point where you cut it
     *   - B cancels, so you are never stuck watching it do the wrong thing
     *   - it times out, so a jammed deflector stops instead of grinding
     */
    private void runToTotal(double target) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < GOTO_TIMEOUT_S) {
            updateTotal();
            double error = target - totalDegrees;

            if (Math.abs(error) <= GOTO_TOLERANCE_DEG) {
                servo.setPower(0);
                testResult = String.format("ARRIVED: target %.0f, stopped at %.1f deg",
                        target, totalDegrees);
                return;
            }

            if (gamepad1.b) {
                servo.setPower(0);
                testResult = String.format("Go-to CANCELLED at %.1f deg (target %.0f)",
                        totalDegrees, target);
                return;
            }

            double speed = Math.abs(error) > GOTO_APPROACH_DEG ? MAX_POWER : GOTO_SLOW_POWER;
            double direction = error > 0 ? 1 : -1;
            if (!POSITIVE_POWER_INCREASES_ANGLE) {
                direction = -direction;
            }
            double power = direction * speed;
            servo.setPower(power);

            telemetry.addLine("GO TO TARGET running (B to stop)");
            telemetry.addData("Target", "%.0f deg", target);
            telemetry.addData("Error", "%+.1f deg%s", error,
                    Math.abs(error) > GOTO_APPROACH_DEG ? "" : "   (slowing down)");
            showReadings(power);
            telemetry.update();
        }

        servo.setPower(0);
        testResult = String.format(
                "Go-to TIMED OUT after %.0f s at %.1f deg (target %.0f). Jammed, or "
                        + "POSITIVE_POWER_INCREASES_ANGLE is set the wrong way.",
                GOTO_TIMEOUT_S, totalDegrees, target);
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
        telemetry.addData("Feedback voltage", "%.3f V  (of %.3f V max)",
                feedback.getVoltage(), feedback.getMaxVoltage());
        telemetry.addData("Angle now", "%.1f deg", lastAngle);
        telemetry.addData("Total (counts turns)", "%.1f deg", totalDegrees);

        // The captured range. Nothing to show until the first reading lands.
        if (maxVolts >= minVolts) {
            double spanVolts = maxVolts - minVolts;
            telemetry.addLine();
            telemetry.addData("Volts seen", "%.3f V  ..  %.3f V", minVolts, maxVolts);
            telemetry.addData("Angle seen", "%.1f deg  ..  %.1f deg",
                    minVolts / feedback.getMaxVoltage() * ASSUMED_SPAN_DEG,
                    maxVolts / feedback.getMaxVoltage() * ASSUMED_SPAN_DEG);
            telemetry.addData("Span", "%.3f V  =  %.1f deg (if the %.0f deg assumption holds)",
                    spanVolts,
                    spanVolts / feedback.getMaxVoltage() * ASSUMED_SPAN_DEG,
                    ASSUMED_SPAN_DEG);
        }
    }
}
