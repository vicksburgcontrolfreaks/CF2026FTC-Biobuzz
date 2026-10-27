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
 *   Triggers      - JOG: right = positive power, left = negative power
 *   Y             - toggle JOG / HOLD (closed loop)
 *   X             - start the homing routine
 *   B             - "the turret is at 0 right now" (zeroes the encoder reference)
 *   D-pad L/R     - HOLD mode: nudge the target angle by NUDGE_DEG
 *   A             - HOLD mode: send the target to 0 (straight ahead)
 *
 * Tune every Turret constant live at panels.bylazar.com while this runs.
 */
@Configurable
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    public static double JOG_SCALE = 0.35;   // trigger 1.0 -> this much power
    public static double NUDGE_DEG = 10.0;

    private final Turret turret = new Turret();

    private boolean yPrev, xPrev, bPrev, aPrev, dpadLeftPrev, dpadRightPrev;

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        turret.init(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        // Register both classes that hold tunable constants: this op mode (JOG_SCALE,
        // NUDGE_DEG) and the subsystem itself (TICKS_PER_DEGREE, kP, limits, ...).
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(turret);

        telemetry.addLine("TURRET TEST");
        telemetry.addLine("Triggers = jog | Y = jog/hold | X = home | B = call this 0");
        telemetry.addLine("Gear ratio (servo revs per turret rev): " + Turret.gearRatio());
        telemetry.addLine("");
        telemetry.addLine("NOT HOMED on start: soft limits are OFF. Jog gently.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean y = gamepad1.y;
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            boolean a = gamepad1.a;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

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
                turret.setTargetAngle(turret.getTargetAngle() + NUDGE_DEG);
            }
            if (dpadLeft && !dpadLeftPrev) {
                turret.setTargetAngle(turret.getTargetAngle() - NUDGE_DEG);
            }

            yPrev = y;
            xPrev = x;
            bPrev = b;
            aPrev = a;
            dpadLeftPrev = dpadLeft;
            dpadRightPrev = dpadRight;

            // Raw power from the triggers. Deliberately NOT deadband-compensated,
            // so you can feel exactly where the servo starts to move.
            double jog = (gamepad1.right_trigger - gamepad1.left_trigger) * JOG_SCALE;
            turret.setRawPower(jog);

            turret.update();

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
