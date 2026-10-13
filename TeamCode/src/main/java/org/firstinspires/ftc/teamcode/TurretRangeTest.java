package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Maps the turret's physical travel by hand, and prints the Turret constants that
 * come out of it. No power is applied -- the servo is explicitly stopped and never
 * driven, so you push the turret yourself.
 *
 * WHY THIS BEATS MEASURING 90 DEGREES WITH A PROTRACTOR
 * Pushing gently to both hard stops captures the whole TRAVEL_DEG of rotation in a
 * single reading. Any slop in where exactly you stopped is spread across 210 degrees
 * instead of 90, so the ticks-per-degree number that falls out is about twice as
 * accurate -- and it needs no protractor at all, just the stops the robot already has.
 *
 * HOW TO USE
 *   1. Press start. Push the turret gently to ONE hard stop, then slowly all the way
 *      to the OTHER, passing the magnet on the way. Do not force past a stop.
 *   2. Read the DERIVED CONSTANTS block and write the numbers down.
 *   3. A resets everything if you want a clean second run.
 *
 * Config needed:
 *   "turretHome" - REV Touch Sensor (magnetic limit switch)
 *   "rf"         - right front drive motor, carrying the turret encoder on its
 *                  spare encoder port
 *   "turret"     - optional. Only fetched so it can be explicitly stopped.
 */
@Configurable
@TeleOp(name = "Turret Range Test", group = "Test")
public class TurretRangeTest extends LinearOpMode {

    /**
     * Total physical travel between the hard stops, in degrees. The team measured
     * 210. Everything below is derived from this, so if it is wrong, all the derived
     * constants are wrong by the same factor -- it is the one number to be sure of.
     */
    public static double TRAVEL_DEG = 210.0;

    private DcMotorEx encoderMotor;
    private TouchSensor homeSwitch;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        encoderMotor = hardwareMap.get(DcMotorEx.class, "rf");
        homeSwitch = hardwareMap.get(TouchSensor.class, "turretHome");

        // Fetched only to guarantee it is not driving while hands are on the turret.
        CRServo servo = hardwareMap.tryGet(CRServo.class, "turret");
        if (servo != null) {
            servo.setPower(0.0);
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetry.addLine("TURRET RANGE TEST - no power applied, turn by hand");
        telemetry.addLine("Push gently to one stop, then slowly to the other.");
        telemetry.addLine("Pass the magnet on the way. A = reset.");
        telemetry.addLine(servo != null
                ? "turret servo found and stopped"
                : "turret servo not configured (fine - nothing to stop)");
        telemetry.update();

        waitForStart();

        int startTicks = readRawTicks();
        int minTicks = startTicks;
        int maxTicks = startTicks;

        boolean haveMagnet = false;
        int magnetMinTicks = 0;
        int magnetMaxTicks = 0;

        boolean aPrev = false;

        while (opModeIsActive()) {
            int ticks = readRawTicks();
            boolean pressed = homeSwitch.isPressed();
            boolean a = gamepad1.a;

            if (a && !aPrev) {
                minTicks = ticks;
                maxTicks = ticks;
                haveMagnet = false;
            }
            aPrev = a;

            minTicks = Math.min(minTicks, ticks);
            maxTicks = Math.max(maxTicks, ticks);

            // Track the extent of the magnet's trigger band, so its CENTER can be
            // located rather than whichever edge happened to be hit first.
            if (pressed) {
                if (!haveMagnet) {
                    magnetMinTicks = ticks;
                    magnetMaxTicks = ticks;
                    haveMagnet = true;
                } else {
                    magnetMinTicks = Math.min(magnetMinTicks, ticks);
                    magnetMaxTicks = Math.max(magnetMaxTicks, ticks);
                }
            }

            int spanTicks = maxTicks - minTicks;

            telemetryM.addLine("=== LIVE ===");
            telemetryM.addData("encoder ticks", ticks);
            telemetryM.addData("magnet", pressed ? "PRESSED" : "open");
            telemetryM.addData("ticks from min end", ticks - minTicks);
            telemetryM.addData("ticks from max end", maxTicks - ticks);

            telemetryM.addLine("");
            telemetryM.addLine("=== TRAVEL SEEN SO FAR ===");
            telemetryM.addData("min ticks", minTicks);
            telemetryM.addData("max ticks", maxTicks);
            telemetryM.addData("span (ticks)", spanTicks);
            telemetryM.addLine("Span stops growing once both stops are reached.");

            telemetryM.addLine("");
            telemetryM.addLine("=== DERIVED CONSTANTS ===");
            if (spanTicks < 50) {
                telemetryM.addLine("Not enough travel yet - keep sweeping.");
            } else {
                double ticksPerDegree = spanTicks / TRAVEL_DEG;
                double halfTravel = TRAVEL_DEG / 2.0;
                double midTicks = (minTicks + maxTicks) / 2.0;

                telemetryM.addData("TRAVEL_DEG (input)", TRAVEL_DEG);
                telemetryM.addData("TICKS_PER_DEGREE", ticksPerDegree);
                telemetryM.addData("MIN_ANGLE", -halfTravel);
                telemetryM.addData("MAX_ANGLE", halfTravel);
                telemetryM.addData("current angle (deg)",
                        toAngle(ticks, midTicks, ticksPerDegree));

                if (haveMagnet) {
                    double magnetCenter = (magnetMinTicks + magnetMaxTicks) / 2.0;
                    double homeAngle = toAngle(magnetCenter, midTicks, ticksPerDegree);
                    double bandDeg = (magnetMaxTicks - magnetMinTicks) / ticksPerDegree;

                    telemetryM.addData("HOME_SWITCH_ANGLE", homeAngle);
                    telemetryM.addData("magnet band (deg)", bandDeg);
                    // Homing has to drive TOWARD the magnet, so the sign follows
                    // whichever side of center the magnet turned out to be on.
                    telemetryM.addData("HOMING_DIRECTION",
                            homeAngle < 0 ? -1.0 : 1.0);
                } else {
                    telemetryM.addLine("HOME_SWITCH_ANGLE: sweep past the magnet");
                }
            }

            telemetryM.addLine("");
            telemetryM.addLine("ENCODER_DIRECTION is " + Turret.ENCODER_DIRECTION
                    + " - angles above use it");

            telemetryM.update(telemetry);
        }
    }

    /** Ticks to turret degrees, zero at the midpoint of measured travel. */
    private double toAngle(double ticks, double midTicks, double ticksPerDegree) {
        return (ticks - midTicks) * Turret.ENCODER_DIRECTION / ticksPerDegree;
    }

    /**
     * Encoder ticks with the drive motor's direction cancelled out, matching
     * Turret.readRawTicks(). The SDK negates getCurrentPosition() on a REVERSE
     * motor, and rf's direction is set inconsistently across op modes.
     */
    private int readRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        if (encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            ticks = -ticks;
        }
        return ticks;
    }
}
