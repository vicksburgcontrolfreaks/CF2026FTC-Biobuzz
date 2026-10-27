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
 * WHAT THIS MEASURES
 * Not TICKS_PER_DEGREE -- that comes from the gear teeth and the encoder's
 * counts/rev, which are exact integers (see Turret). This op mode runs the other
 * way round: it takes Turret.TICKS_PER_DEGREE as known and reports the TRAVEL the
 * sweep implies, plus the magnet's position and the soft limits that follow.
 * Pushing gently to both hard stops captures the whole travel in one reading, using
 * stops the robot already has instead of a protractor.
 *
 * HOW TO USE
 *   1. Press start. Push the turret gently to ONE hard stop, then slowly all the way
 *      to the OTHER, passing the magnet on the way. Do not force past a stop.
 *   2. Read the DERIVED CONSTANTS block and write the numbers down. Sweep
 *      straight through the magnet without backing up, or the band reads wrong.
 *   3. A resets everything if you want a clean second run.
 *
 * Config needed:
 *   "turretHome" - REV Touch Sensor (magnetic limit switch)
 *   "lf"         - left front drive motor, carrying the turret encoder on its
 *                  spare encoder port
 *   "turret"     - optional. Only fetched so it can be explicitly stopped.
 */
@Configurable
@TeleOp(name = "Turret Range Test", group = "Test")
public class TurretRangeTest extends LinearOpMode {

    /**
     * Optional cross-check, ignored when 0. If you think you know the travel from
     * somewhere else -- CAD, a protractor -- put it here and this op mode will show
     * the ticks-per-degree that that travel would imply, next to the one the gear
     * teeth give. If the two disagree, one of the two inputs is wrong, and the gear
     * teeth are integers.
     */
    public static double TRAVEL_DEG_CHECK = 0.0;

    private DcMotorEx encoderMotor;
    private TouchSensor homeSwitch;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        encoderMotor = hardwareMap.get(DcMotorEx.class, "lf");
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

            double tpd = Turret.TICKS_PER_DEGREE;
            double midTicks = (minTicks + maxTicks) / 2.0;

            // Angle first: it is the number you came here to read, and the Driver
            // Station telemetry window cuts off long blocks.
            telemetryM.addLine("=== LIVE ===");
            telemetryM.addData("angle (deg)", toAngle(ticks, midTicks, tpd));
            telemetryM.addData("encoder ticks", ticks);
            telemetryM.addData("magnet", pressed ? "PRESSED" : "open");
            telemetryM.addLine("Angle zero is the middle of the sweep SO FAR, so it");
            telemetryM.addLine("only settles once both stops have been touched.");

            telemetryM.addLine("");
            telemetryM.addLine("=== TRAVEL ===");
            telemetryM.addData("span (ticks)", spanTicks);
            telemetryM.addData("travel (deg)", spanTicks / tpd);
            telemetryM.addData("min / max ticks", minTicks + " / " + maxTicks);
            telemetryM.addLine("Both stop growing once both stops are reached.");

            telemetryM.addLine("");
            telemetryM.addLine("=== DERIVED CONSTANTS ===");
            if (spanTicks < 50) {
                telemetryM.addLine("Not enough travel yet - keep sweeping.");
            } else {
                double halfTravel = spanTicks / tpd / 2.0;

                telemetryM.addData("hard stops (deg)",
                        String.format("%.1f / +%.1f", -halfTravel, halfTravel));
                telemetryM.addData("suggested MIN_ANGLE", -(halfTravel - 3.0));
                telemetryM.addData("suggested MAX_ANGLE", halfTravel - 3.0);

                if (haveMagnet) {
                    double loEdge = toAngle(magnetMinTicks, midTicks, tpd);
                    double hiEdge = toAngle(magnetMaxTicks, midTicks, tpd);
                    double bandDeg = Math.abs(hiEdge - loEdge);
                    double bandCenter = (loEdge + hiEdge) / 2.0;

                    // Homing always makes its final approach travelling in
                    // HOMING_DIRECTION, so it trips on whichever edge it meets
                    // FIRST coming that way: the high-angle edge when travelling
                    // negative, the low-angle edge when travelling positive. That
                    // one edge -- not the band center -- is the reference.
                    boolean approachingNegative = Turret.HOMING_DIRECTION < 0;
                    double approachEdge = approachingNegative
                            ? Math.max(loEdge, hiEdge)
                            : Math.min(loEdge, hiEdge);

                    telemetryM.addData("HOME_SWITCH_ANGLE", approachEdge);
                    telemetryM.addLine("^ the edge homing meets first travelling "
                            + (approachingNegative ? "-" : "+"));
                    telemetryM.addData("magnet band (deg)", bandDeg);
                    telemetryM.addData("  band edges", String.format("%.1f / %.1f", loEdge, hiEdge));
                    telemetryM.addData("  band center (FYI)", bandCenter);
                    telemetryM.addLine("Band only reads right if you swept STRAIGHT");
                    telemetryM.addLine("through the magnet without backing up.");
                    // Homing has to drive TOWARD the magnet, so the sign follows
                    // whichever side of center the magnet turned out to be on.
                    telemetryM.addData("HOMING_DIRECTION", bandCenter < 0 ? -1.0 : 1.0);
                } else {
                    telemetryM.addLine("HOME_SWITCH_ANGLE: sweep past the magnet");
                }

                // Only when someone is actively arguing with the gear-teeth number.
                if (TRAVEL_DEG_CHECK > 0.0) {
                    telemetryM.addLine("");
                    telemetryM.addLine("=== CROSS-CHECK ===");
                    telemetryM.addData("if travel really were (deg)", TRAVEL_DEG_CHECK);
                    telemetryM.addData("  then TICKS_PER_DEGREE is", spanTicks / TRAVEL_DEG_CHECK);
                    telemetryM.addData("  but the gear teeth say", tpd);
                }
            }

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
     * motor, and lf's direction is set inconsistently across op modes.
     */
    private int readRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        if (encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            ticks = -ticks;
        }
        return ticks;
    }
}
