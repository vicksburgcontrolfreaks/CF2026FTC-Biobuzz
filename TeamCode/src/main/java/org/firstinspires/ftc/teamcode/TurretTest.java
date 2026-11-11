package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Bring-up and tuning op mode for the Turret subsystem.
 *
 * Run this BEFORE putting the turret in a real TeleOp. It exists to answer four
 * questions you cannot answer from a datasheet:
 *   1. Which way does the encoder count when the servo gets positive power?
 *   2. How many encoder ticks is one real degree of turret rotation?
 *   3. What is the lowest power that actually moves the turret (the deadband)?
 *   4. Where does the magnet switch trip, and does homing find it?
 *
 * Controls (gamepad1):
 *   Triggers      - JOG: left = swing left (CCW, angle up), right = swing right
 *   Y             - toggle JOG / HOLD (closed loop)
 *   X             - start the homing routine
 *   B             - "the turret is at 0 right now" (zeroes the encoder reference)
 *   D-pad L/R     - HOLD mode: nudge the target NUDGE_DEG left (CCW) / right (CW)
 *   Bumpers L/R   - HOLD mode: fine nudge, FINE_NUDGE_DEG left / right
 *   After homing succeeds the turret is in HOLD at 0; press Y to jog again.
 *   A             - HOLD mode: send the target to 0 (straight ahead)
 *
 * Tune every Turret constant live at panels.bylazar.com while this runs.
 */
@Configurable
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    public static double JOG_SCALE = 0.35;   // trigger 1.0 -> this much power
    public static double NUDGE_DEG = 10.0;
    // = Turret.TOLERANCE_DEG: a smaller step can sit inside the "close enough" band
    // and not move the turret at all.
    public static double FINE_NUDGE_DEG = 2.0;

    private final Turret turret = new Turret();

    private boolean yPrev, xPrev, bPrev, aPrev, dpadLeftPrev, dpadRightPrev, lbPrev, rbPrev;

    // ---- Settle measurement (added 2026-11-11) ------------------------------
    // Answers one question: can this turret settle finely, or does the minimum
    // power step throw it past the target?
    //
    // MIN_POWER / kP = 0.12 / 0.020 = 6 deg on 6494. The loop returns zero inside
    // TOLERANCE_DEG (2 deg) and otherwise boosts anything below MIN_POWER UP to
    // MIN_POWER, so the smallest correction it can command is sized for 6 deg of
    // error while it is trying to land in a 2 deg band. If that overshoots, the
    // turret cannot sit still and the aim wanders a few degrees at random --
    // which is what Shane saw aiming at the hive on 2026-11-11.
    //
    // All three reset on every target change, so each nudge is its own clean test.
    private double lastAngle = Double.NaN;
    private double lastTarget = Double.NaN;
    private double minPowerStepDeg = 0;   // furthest it moved in ONE loop at MIN_POWER
    private int crossings = 0;            // times the error changed sign = overshoots
    private double maxOvershootDeg = 0;   // furthest past the target it got
    private double startErrorSign = 0;
    private double lastSign = 0;

    private TelemetryManager telemetryM;

    /**
     * Watches one nudge settle. Everything resets when the target moves, so each
     * bumper press is a fresh measurement.
     *
     * READING IT: nudge 2 deg with a bumper, wait for it to stop, then
     *   Overshoots 0        -> it CAN settle finely. MIN_POWER is not the problem,
     *                          and the random aim wander is pose noise instead.
     *   Overshoots 1 or 2   -> rings once and stops. Liveable.
     *   Overshoots climbing -> it cannot sit still. The minimum power step is
     *                          throwing it past the band every time, and
     *                          LOWERING TOLERANCE_DEG WOULD MAKE IT WORSE.
     *                          Lower MIN_POWER instead, as far as the servo will
     *                          still reliably turn.
     *
     * Min step says the same thing in travel terms: under about 1 deg per loop and
     * fine settling is physically possible; several degrees and it is not.
     */
    private void measureSettle() {
        double angle = turret.getAngle();
        double target = turret.getTargetAngle();
        double error = target - angle;

        if (target != lastTarget) {
            minPowerStepDeg = 0;
            crossings = 0;
            maxOvershootDeg = 0;
            startErrorSign = Math.signum(error);
            lastSign = startErrorSign;
            lastTarget = target;
            lastAngle = angle;
            return;   // nothing has travelled yet on the press itself
        }

        // Closed loop only, and only while the boost is what is actually being
        // applied -- jog power is whatever the trigger says and means nothing here.
        if (turret.getControlMode() == Turret.ControlMode.HOLD && !Double.isNaN(lastAngle)
                && Math.abs(Math.abs(turret.getAppliedPower()) - Turret.MIN_POWER) < 0.005) {
            minPowerStepDeg = Math.max(minPowerStepDeg, Math.abs(angle - lastAngle));
        }

        // Past the target means the error flipped against the way it started.
        double sign = Math.signum(error);
        if (startErrorSign != 0 && sign != 0 && sign != startErrorSign) {
            maxOvershootDeg = Math.max(maxOvershootDeg, Math.abs(error));
        }
        if (sign != 0 && lastSign != 0 && sign != lastSign) {
            crossings++;
        }
        if (sign != 0) lastSign = sign;

        lastAngle = angle;
    }

    private static String fmtOrDash(double v) {
        return v <= 0 ? "--" : String.format("%.2f", v);
    }

    @Override
    public void runOpMode() {
        turret.init(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        // Register both classes that hold tunable constants: this op mode (JOG_SCALE,
        // NUDGE_DEG) and the subsystem itself (TICKS_PER_DEGREE, kP, limits, ...).
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(turret);

        telemetry.addLine("TURRET TEST");
        telemetry.addLine("Triggers = jog (L = left, R = right) | Y = jog/hold | D-pad 10, bumpers 2 deg | X = home | B = call this 0");
        telemetry.addLine("MIN_POWER settle test: Y to HOLD, then a BUMPER for a 2 deg nudge. Read Overshoots.");
        telemetry.addLine("Gear ratio (servo revs per turret rev): " + Turret.gearRatio());
        telemetry.addLine("");
        telemetry.addLine(turret.wasHomingKept()
                ? "HOMED from an earlier op mode (HOLD). X re-homes; Y to jog."
                : "NOT HOMED on start: soft limits are OFF. Jog gently.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean y = gamepad1.y;
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            boolean a = gamepad1.a;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (y && !yPrev) {
                turret.setControlMode(turret.getControlMode() == Turret.ControlMode.JOG
                        ? Turret.ControlMode.HOLD
                        : Turret.ControlMode.JOG);
            }
            if (x && !xPrev) {
                turret.startHoming();
            }
            if (b && !bPrev) {
                // Line the turret up straight ahead by eye, then press B. Now the
                // angle readout is real, and you can measure ticks per degree.
                turret.resetReferenceHere(0.0);
            }
            if (a && !aPrev) {
                turret.setTargetAngle(0.0);
            }
            if (dpadRight && !dpadRightPrev) {
                turret.setTargetAngle(turret.getTargetAngle() - NUDGE_DEG);
            }
            if (dpadLeft && !dpadLeftPrev) {
                turret.setTargetAngle(turret.getTargetAngle() + NUDGE_DEG);
            }
            if (rb && !rbPrev) {
                turret.setTargetAngle(turret.getTargetAngle() - FINE_NUDGE_DEG);
            }
            if (lb && !lbPrev) {
                turret.setTargetAngle(turret.getTargetAngle() + FINE_NUDGE_DEG);
            }

            yPrev = y;
            xPrev = x;
            bPrev = b;
            aPrev = a;
            dpadLeftPrev = dpadLeft;
            dpadRightPrev = dpadRight;
            lbPrev = lb;
            rbPrev = rb;

            // Raw power from the triggers. Deliberately NOT deadband-compensated,
            // so you can feel exactly where the servo starts to move.
            // Left = swing left (CCW from above, angle up); right = swing right.
            // Matches the D-pad below. Shane, 6494, 2026-11-06.
            double jog = (gamepad1.left_trigger - gamepad1.right_trigger) * JOG_SCALE;
            turret.setRawPower(jog);

            turret.update();
            measureSettle();

            telemetryM.addData("State", turret.getState());
            telemetryM.addData("Mode", turret.getControlMode());
            telemetryM.addData("Note", turret.getStatusNote());
            telemetryM.addData("Home switch", turret.isHomeSwitchPressed() ? "PRESSED" : "open");
            telemetryM.addLine("");
            telemetryM.addData("Angle (deg)", turret.getAngle());
            telemetryM.addData("Target (deg)", turret.getTargetAngle());
            telemetryM.addData("On target", turret.isOnTarget());
            telemetryM.addLine("");
            telemetryM.addData("Raw ticks", turret.getRawTicks());
            telemetryM.addData("Ticks since ref", turret.getTicksSinceReference());
            telemetryM.addData("Commanded power", turret.getAppliedPower());
            telemetryM.addLine("");
            // The MIN_POWER settle test. Nudge 2 deg with a bumper and read these.
            telemetryM.addData("Min step (deg/loop at MIN_POWER)", fmtOrDash(minPowerStepDeg));
            telemetryM.addData("Overshoots since nudge", crossings
                    + (crossings == 0 ? "  (settled clean)" : "  worst " + String.format("%.2f", maxOvershootDeg) + " deg past"));
            telemetryM.addData("Min step is worth (deg of error)",
                    Turret.kP > 0 ? String.format("%.1f", Turret.MIN_POWER / Turret.kP) : "?");
            telemetryM.addData("Jog request", jog);

            if (!turret.isHomed()) {
                telemetryM.addLine("");
                telemetryM.addLine("NOT HOMED - angle is a guess, soft limits OFF");
            }
            // Passing the Driver Station telemetry mirrors everything to both places.
            telemetryM.update(telemetry);
        }

        turret.stop();
    }
}
