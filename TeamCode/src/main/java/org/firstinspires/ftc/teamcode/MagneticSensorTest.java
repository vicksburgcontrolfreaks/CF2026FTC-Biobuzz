package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Standalone test for the turret's magnetic limit switch. Run this BEFORE trusting
 * the homing routine in TurretTest, because homing drives the turret at power
 * looking for this switch -- if the signal is backwards or flaky, homing either
 * stops instantly at the wrong angle or never stops at all.
 *
 * It answers four questions:
 *   1. POLARITY  - does it read pressed when the magnet is near, or the opposite?
 *   2. RELIABILITY - does it trip cleanly, or chatter between states?
 *   3. CATCHES A SWEEP - does it still register when the magnet goes by fast?
 *   4. ZONE WIDTH - over how much turret travel does it stay pressed?
 *
 * Nothing on the robot moves in this op mode -- turn the turret by hand.
 *
 * Config needed:
 *   "turretHome" - REV Touch Sensor (the magnetic limit switch), digital port
 *   "rf"         - optional. The right front drive motor, whose spare encoder port
 *                  carries the turret encoder. If it is configured, this op mode
 *                  also measures how wide the magnet's trigger zone is. Without
 *                  it, everything except the zone measurement still works.
 *
 * Controls: A resets all the counters and the zone measurement.
 */
@TeleOp(name = "Magnetic Sensor Test", group = "Test")
public class MagneticSensorTest extends LinearOpMode {

    private TouchSensor homeSwitch;
    private DcMotorEx encoderMotor;   // may be null -- read only, never driven

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        homeSwitch = hardwareMap.get(TouchSensor.class, "turretHome");

        // tryGet returns null instead of throwing, so this op mode still runs on a
        // bench setup where the drivetrain is not configured.
        encoderMotor = hardwareMap.tryGet(DcMotorEx.class, "rf");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("MAGNETIC SENSOR TEST");
        telemetry.addLine("Nothing moves. Turn the turret by hand.");
        telemetry.addLine("A = reset counters");
        telemetry.addLine(encoderMotor != null
                ? "rf encoder found: zone width will be measured"
                : "rf not configured: zone width unavailable (everything else works)");
        telemetry.update();

        waitForStart();

        boolean lastPressed = homeSwitch.isPressed();
        int pressEvents = 0;
        int releaseEvents = 0;
        boolean everPressed = lastPressed;
        boolean aPrev = false;

        // Encoder extremes seen while the switch was pressed -- the trigger zone.
        boolean haveZone = false;
        int zoneMinTicks = 0;
        int zoneMaxTicks = 0;

        final ElapsedTime stateTimer = new ElapsedTime();

        while (opModeIsActive()) {
            boolean pressed = homeSwitch.isPressed();
            boolean a = gamepad1.a;

            if (a && !aPrev) {
                pressEvents = 0;
                releaseEvents = 0;
                everPressed = pressed;
                haveZone = false;
                stateTimer.reset();
            }
            aPrev = a;

            if (pressed != lastPressed) {
                if (pressed) {
                    pressEvents++;
                    everPressed = true;
                } else {
                    releaseEvents++;
                }
                lastPressed = pressed;
                stateTimer.reset();
            }

            // Widen the measured zone for as long as the switch stays pressed. Using
            // min/max rather than the press/release edges means it works no matter
            // which direction the turret is swinging.
            if (pressed && encoderMotor != null) {
                int ticks = readRawTicks();
                if (!haveZone) {
                    zoneMinTicks = ticks;
                    zoneMaxTicks = ticks;
                    haveZone = true;
                } else {
                    zoneMinTicks = Math.min(zoneMinTicks, ticks);
                    zoneMaxTicks = Math.max(zoneMaxTicks, ticks);
                }
            }

            telemetryM.addLine("=== RIGHT NOW ===");
            telemetryM.addData("isPressed()", pressed ? "PRESSED" : "open");
            telemetryM.addData("getValue()", homeSwitch.getValue());
            telemetryM.addData("held this state for (s)", stateTimer.seconds());

            telemetryM.addLine("");
            telemetryM.addLine("=== POLARITY ===");
            telemetryM.addLine("Magnet AWAY should read 'open'.");
            telemetryM.addLine("Magnet NEAR should read 'PRESSED'.");
            telemetryM.addLine(pressed
                    ? "-> magnet should be NEAR right now"
                    : "-> magnet should be AWAY right now");

            telemetryM.addLine("");
            telemetryM.addLine("=== RELIABILITY ===");
            telemetryM.addData("press events", pressEvents);
            telemetryM.addData("release events", releaseEvents);
            telemetryM.addData("ever pressed", everPressed ? "yes" : "NO - never saw it");
            telemetryM.addLine("One pass of the magnet = 1 press + 1 release.");
            telemetryM.addLine("Many more than that means it is chattering.");

            telemetryM.addLine("");
            telemetryM.addLine("=== TRIGGER ZONE ===");
            if (encoderMotor == null) {
                telemetryM.addLine("rf not configured - skipped");
            } else if (!haveZone) {
                telemetryM.addData("encoder ticks", readRawTicks());
                telemetryM.addLine("Swing the magnet through to measure.");
            } else {
                int widthTicks = zoneMaxTicks - zoneMinTicks;
                telemetryM.addData("encoder ticks", readRawTicks());
                telemetryM.addData("zone width (ticks)", widthTicks);
                // Approximate until TICKS_PER_DEGREE has actually been measured
                // on the robot -- see docs/turret-bringup.md step 4.
                telemetryM.addData("zone width (deg, approx)",
                        widthTicks / Turret.TICKS_PER_DEGREE);
                telemetryM.addLine("A wide zone means the homed zero shifts with");
                telemetryM.addLine("approach speed. Narrow is better.");
            }

            telemetryM.update(telemetry);
        }
    }

    /**
     * Encoder ticks with the drive motor's direction cancelled out, matching what
     * Turret.readRawTicks() does -- the SDK negates getCurrentPosition() on a
     * REVERSE motor, and rf's direction is set inconsistently across op modes.
     */
    private int readRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        if (encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            ticks = -ticks;
        }
        return ticks;
    }
}
