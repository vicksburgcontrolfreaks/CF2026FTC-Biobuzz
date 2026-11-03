package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Servo Position Test — use the joystick to find the positions you want.
 *
 * How to use:
 *   1. In the Driver Station robot config, name your servo "testServo"
 *      (or change SERVO_NAME below to match the name it already has).
 *   2. Run this op mode. The servo moves to START_POSITION when you press START.
 *   3. Left stick up/down  = move the servo (push farther = move faster)
 *      D-pad up/down       = tiny steps, for fine-tuning
 *   4. When it's where you want it, read "Position" off the Driver Station
 *      screen and write the number down.
 *
 * Servo positions go from 0.0 to 1.0 — that's the servo's whole range of travel.
 */
@TeleOp(name = "Servo Position Test", group = "Test")
public class ServoPositionTest extends LinearOpMode {

    // Change this to match the servo's name in the robot configuration.
    private static final String SERVO_NAME = "testServo";

    // Where the servo goes when you press START. 0.5 is the middle.
    // CAREFUL: if the servo is attached to a mechanism that can't reach the
    // middle (it would hit something), change this to a safe position first!
    private static final double START_POSITION = 0.5;

    // How fast the stick moves the servo, in position units per second.
    // 0.25 means full stick takes 4 seconds to go from 0.0 to 1.0.
    private static final double STICK_SPEED = 0.25;

    // How far one D-pad press moves the servo.
    private static final double DPAD_STEP = 0.01;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);
        double position = START_POSITION;

        // Used to remember if the D-pad was already held last loop, so one
        // press = one step (instead of zooming while you hold it).
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;

        telemetry.addLine("Ready. Servo will move to " + START_POSITION + " when you press START.");
        telemetry.update();

        waitForStart();

        // A timer so the stick moves the servo at the same speed no matter
        // how fast the loop runs.
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double secondsSinceLastLoop = loopTimer.seconds();
            loopTimer.reset();

            // Stick up is NEGATIVE on gamepads, so flip it with a minus sign.
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.05) {   // ignore tiny drift when the stick is "centered"
                position += stick * STICK_SPEED * secondsSinceLastLoop;
            }

            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                position += DPAD_STEP;
            }
            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                position -= DPAD_STEP;
            }
            dpadUpWasPressed = gamepad1.dpad_up;
            dpadDownWasPressed = gamepad1.dpad_down;

            // Keep it between 0.0 and 1.0 — the servo can't go past those.
            position = Range.clip(position, 0.0, 1.0);
            servo.setPosition(position);

            telemetry.addData("Position", "%.3f", position);
            telemetry.addLine();
            telemetry.addLine("Left stick: move   |   D-pad up/down: tiny steps");
            telemetry.update();
        }
    }
}
