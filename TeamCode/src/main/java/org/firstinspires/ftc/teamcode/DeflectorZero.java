package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

/**
 * Deflector Zero -- run BEFORE EVERY MATCH (and after any redeploy or power
 * cycle). Stand-in until the team's own pre-match op mode is done; that one
 * only needs Deflector.zeroHere() too.
 *
 * Why every match: the deflector's feedback can't tell which of its ~7.5 turns
 * it's on, so after power-up nothing knows its angle and it refuses to move.
 * Zeroing tells it "this is level". The zero then carries through auto and
 * TeleOp until the next redeploy or power cycle.
 *
 * Controls (gamepad1), after START:
 *   Left stick Y   jog it slowly (up = up). Use this to get it LEVEL. No soft
 *                  limits here (an old zero could be wrong) -- watch it, and
 *                  stop at the frame.
 *   A              "it's level right now" -> 0 deg. Rumbles.
 *   D-pad up/down  step through the positions (0/67.5/70/72.5/75/80/90) to check the zero
 *   B              forget the zero (it stops responding until A again)
 *
 * HOW
 *   1. START. Jog it until it's parallel to the floor -- use a phone level on
 *      the deflector, not your eye.
 *   2. Press A. "Zeroed" turns YES and it holds at 0.
 *   3. Optional check: D-pad up to 90 -- it should be straight up. Then come
 *      back down to 0 and leave it there (or wherever the match should start).
 *   4. Stop this op mode and start auto. Do NOT redeploy or power cycle now --
 *      that throws the zero away.
 */
@TeleOp(name = "Deflector Zero", group = "Setup")
public class DeflectorZero extends LinearOpMode {

    private static final double JOG_POWER = 0.3;   // full stick; slow on purpose

    @Override
    public void runOpMode() {
        Deflector deflector = new Deflector(hardwareMap);
        // The old zero may be wrong -- that's why we're here. Its soft limits
        // must not stop the jog (2026-11-06: an off zero walled it off ~10 deg up).
        deflector.setJogIgnoresLimits(true);

        while (opModeInInit()) {
            deflector.update(false);   // track only -- nothing moves during INIT
            if (!deflector.isPresent()) {
                telemetry.addLine("No deflector in this config: needs CR servo \""
                        + Deflector.SERVO_NAME + "\" and analog input \"" + Deflector.FEEDBACK_NAME + "\".");
            } else {
                telemetry.addLine("DEFLECTOR ZERO - press START, jog it LEVEL, press A.");
                show(deflector);
            }
            telemetry.update();
        }
        if (!deflector.isPresent()) return;

        boolean upPrev = false, downPrev = false;
        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;   // up = +
            deflector.jog(Math.abs(stick) > 0.1 ? stick * JOG_POWER : 0);

            if (gamepad1.aWasPressed()) {
                deflector.zeroHere();
                gamepad1.rumble(300);
            }
            if (gamepad1.bWasPressed()) deflector.clearZero();

            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (deflector.isZeroed()) {
                if (up && !upPrev) deflector.stepUp();
                if (down && !downPrev) deflector.stepDown();
            }
            upPrev = up;
            downPrev = down;

            deflector.update(true);

            telemetry.addLine("Stick: jog   A: zero here (LEVEL)   D-pad: step   B: forget zero");
            show(deflector);
            telemetry.update();
        }
        deflector.stop();
    }

    private void show(Deflector d) {
        telemetry.addData("Zeroed", d.isZeroed() ? "YES" : "no - won't hold a position");
        if (d.isZeroed()) {
            telemetry.addData("Angle", "%.1f deg   target %.1f (position %d of %d)%s",
                    d.getAngle(), d.getTarget(), d.getIndex() + 1, DeflectorLogic.POSITIONS.length,
                    d.isOnTarget() ? "  ON TARGET" : "");
            // Auto-home measurement (2026-11-08): zero at level, jog DOWN onto the
            // hard stop, and this is the number for Deflector.loadRobotNumbers().
            telemetry.addData("On the hard stop? LEVEL_ABOVE_STOP_DEG", "%.1f", -d.getAngle());
        }
        telemetry.addData("Servo total", "%.0f deg  (2687 = 90 deg of deflector)", d.getServoTotal());
        telemetry.addData("Power", "%.2f   max step %.0f/180 servo deg per loop", d.getPower(), d.getMaxStepSeen());
    }
}
