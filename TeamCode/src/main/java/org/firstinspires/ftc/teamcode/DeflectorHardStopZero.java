package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

/**
 * Deflector Hard Stop Zero -- EXPERIMENTAL alternative to Deflector Zero.
 *
 * Deflector Zero makes a PERSON decide "it's level" and press A. This instead
 * drives the deflector down (negative) on its own until it stalls against its
 * own mechanical hard stop, calls THAT position zero, then drives back up to
 * wherever it was sitting the moment START was pressed.
 *
 * So: during INIT, move the deflector BY HAND to the angle you want it to
 * come back to (it's unpowered and the feedback tracks passively, same as
 * Deflector Zero's INIT). Press START. It remembers that spot, drives down to
 * find the hard stop, zeros there, then drives back up to it -- proving the
 * hard-stop zero actually lands where you expect.
 *
 * NOT yet the match zeroing routine -- Deflector Zero (level + A) is what
 * Select Auto and DriveTeleOp are built around (docs/TODO.md, 2026-11-07, on
 * why there is only ever ONE Deflector class and zeroing convention). This
 * exists to find out whether hard-stop zeroing is repeatable enough to
 * replace that: press A again once it's holding and compare "Zero Drift"
 * across a few runs, same idea as Shooter Pivot Home Test's drift check.
 *
 * Controls (gamepad1), after START:
 *   (automatic)     drives down, detects the stall, zeros there, then
 *                   drives back up to the angle it started at
 *   A               once holding (or timed out): run the sequence again
 *                   (returns to the SAME starting angle, not wherever it is now)
 *   D-pad up/down   once zeroed: step through DeflectorLogic.POSITIONS
 *   B               stop now and hold wherever it is
 *
 * TUNING (watch "Servo total" during the down phase to pick these):
 *   Never detects the stall -> STALL_EPSILON_DEG is smaller than the
 *     feedback's own noise, so it never looks "still". Raise it.
 *   Declares the stall too early (stops short of the real stop) ->
 *     STALL_WINDOW_MS is too short. Raise it.
 *   Never reaches the stop at all -> DOWN_POWER is too gentle, or
 *     DOWN_TIMEOUT_MS is too short for how far down the stop actually is.
 */
@TeleOp(name = "Deflector Hard Stop Zero", group = "Test")
public class DeflectorHardStopZero extends LinearOpMode {

    private static final double DOWN_POWER = -0.15;       // slow and controlled into the stop
    private static final double STALL_EPSILON_DEG = 1.0;  // servo degrees; below this = "not moving"
    private static final long STALL_WINDOW_MS = 300;      // this quiet = it's hit the stop
    private static final long DOWN_TIMEOUT_MS = 4000;     // give up if no stall is ever seen

    private enum State { DRIVING_DOWN, DRIVING_TO_TARGET, HOLDING, TIMED_OUT }

    @Override
    public void runOpMode() {
        Deflector deflector = new Deflector(hardwareMap);
        // A stale zero from a previous op mode must not wall off the drive down.
        deflector.setJogIgnoresLimits(true);

        while (opModeInInit()) {
            deflector.update(false);   // track only -- nothing moves during INIT
            if (!deflector.isPresent()) {
                telemetry.addLine("No deflector in this config: needs CR servo \""
                        + Deflector.SERVO_NAME + "\" and analog input \"" + Deflector.FEEDBACK_NAME + "\".");
            } else {
                telemetry.addLine("DEFLECTOR HARD STOP ZERO");
                telemetry.addLine("Move it BY HAND to the angle it should return to, then press START.");
                telemetry.addData("Servo total (unzeroed)", "%.1f deg", deflector.getServoTotal());
            }
            telemetry.update();
        }
        if (!deflector.isPresent()) return;

        // Wherever it's sitting right now (set by hand during INIT) is what it
        // drives back up to after the hard-stop zero. Fixed for the whole op
        // mode -- A re-runs the sequence back to this SAME spot, not wherever
        // it ended up last attempt.
        double startTotal = deflector.getServoTotal();

        State state = State.DRIVING_DOWN;
        ElapsedTime stallTimer = new ElapsedTime();
        ElapsedTime downTimer = new ElapsedTime();
        double lastTotal = deflector.getServoTotal();
        int attempt = 1;
        double lastZeroTotal = Double.NaN;
        double zeroDrift = Double.NaN;
        double returnAngleDeg = Double.NaN;

        boolean aPrev = false, upPrev = false, downPrev = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;

            if (gamepad1.b) {
                // Hold wherever it actually is right now, not whatever the old target was.
                if (deflector.isZeroed()) {
                    deflector.goTo(deflector.getAngle());
                } else {
                    deflector.jog(0);
                }
                state = State.HOLDING;
            }

            switch (state) {
                case DRIVING_DOWN: {
                    double total = deflector.getServoTotal();
                    if (Math.abs(total - lastTotal) > STALL_EPSILON_DEG) {
                        stallTimer.reset();
                        lastTotal = total;
                    }
                    if (stallTimer.milliseconds() > STALL_WINDOW_MS) {
                        deflector.jog(0);
                        double totalAtZero = deflector.getServoTotal();
                        deflector.zeroHere();   // "here" = the hard stop, not level
                        if (!Double.isNaN(lastZeroTotal)) {
                            zeroDrift = Math.abs(totalAtZero - lastZeroTotal);
                        }
                        lastZeroTotal = totalAtZero;
                        // Convert the INIT-time starting spot into an angle against
                        // the zero we JUST found, and go back to it.
                        returnAngleDeg = DeflectorLogic.toDeflectorDeg(startTotal, totalAtZero);
                        deflector.goTo(returnAngleDeg);
                        state = State.DRIVING_TO_TARGET;
                    } else if (downTimer.milliseconds() > DOWN_TIMEOUT_MS) {
                        deflector.jog(0);
                        state = State.TIMED_OUT;
                    } else {
                        deflector.jog(DOWN_POWER);
                    }
                    break;
                }
                case DRIVING_TO_TARGET: {
                    if (deflector.isOnTarget()) {
                        state = State.HOLDING;
                    }
                    break;
                }
                case HOLDING: {
                    if (up && !upPrev) deflector.stepUp();
                    if (down && !downPrev) deflector.stepDown();
                    // fall through to the A-restart check below
                }
                case TIMED_OUT: {
                    if (a && !aPrev) {
                        attempt++;
                        downTimer.reset();
                        stallTimer.reset();
                        lastTotal = deflector.getServoTotal();
                        state = State.DRIVING_DOWN;
                    }
                    break;
                }
            }

            aPrev = a;
            upPrev = up;
            downPrev = down;

            deflector.update(true);

            telemetry.addData("State", state);
            telemetry.addData("Attempt", attempt);
            telemetry.addData("Return-to angle", Double.isNaN(returnAngleDeg)
                    ? "not computed yet (zero not found)" : String.format("%.1f deg", returnAngleDeg));
            telemetry.addLine();
            telemetry.addData("Zeroed", deflector.isZeroed() ? "YES (at hard stop)" : "no");
            if (deflector.isZeroed()) {
                telemetry.addData("Angle", "%.1f deg   target %.1f (position %d of %d)%s",
                        deflector.getAngle(), deflector.getTarget(), deflector.getIndex() + 1,
                        DeflectorLogic.POSITIONS.length, deflector.isOnTarget() ? "  ON TARGET" : "");
                telemetry.addData("Zero Drift (want < 0.005)",
                        Double.isNaN(zeroDrift) ? "run again (A) to measure" : zeroDrift);
            }
            telemetry.addLine();
            telemetry.addData("Servo total", "%.1f deg", deflector.getServoTotal());
            telemetry.addData("Down timer", "%.0f ms", downTimer.milliseconds());
            telemetry.addData("Stall timer", "%.0f ms (trips at %d)", stallTimer.milliseconds(), STALL_WINDOW_MS);
            telemetry.addData("Power", "%.2f", deflector.getPower());
            telemetry.update();
        }
        deflector.stop();
    }
}
