package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

/**
 * Deflector Auto Home -- find the hard stop by itself, and MEASURE how far above
 * it level sits. One job, nothing else, so it can be proved before any of it
 * gets wired into a match path.
 *
 * WHY THIS EXISTS AS ITS OWN OP MODE
 * Auto-homing was switched on globally and added to Pollen Shot Tune in the same
 * change (2026-11-10). When the zero came out wrong there was nothing to isolate:
 * Deflector Zero behaved differently, the tuner's zeroing behaved differently,
 * and every angle downstream was suspect. That change is reverted. This is the
 * same idea built the other way round -- prove the homing here, then move it.
 *
 * IT TOUCHES NO SHARED DEFAULTS. It never calls Deflector.startHoming() and never
 * writes Deflector.LEVEL_ABOVE_STOP_DEG, so autos and TeleOp keep refusing to
 * auto-home exactly as they do today. The seek loop lives here, in constants you
 * can see, and only moves to the subsystem once the numbers are right.
 *
 * WHAT YOU GET OUT OF IT
 * The number nobody has: LEVEL_ABOVE_STOP_DEG. The stop is a repeatable physical
 * reference and level is a judgement, so measuring the gap ONCE from the stop
 * beats eyeballing level every match. Home, jog to level, press X, write down
 * what it prints.
 *
 * HOW IT FINDS THE STOP
 * Drive down gently and watch the angle. When it stops changing while still being
 * driven, it is against the stop. SETTLE_SEC ignores the first moment (nothing has
 * started moving yet), and the move has to be under STALL_DEG across
 * STALL_WINDOW_SEC to count -- a threshold, not just "did not increase", so a slow
 * patch or a noisy reading is not mistaken for the stop.
 *
 * IT PUSHES INTO A HARD STOP ON PURPOSE. That is what stall detection means. Low
 * power, a short window and a timeout keep it brief -- but watch the mechanism
 * the first few runs, and B stops everything.
 *
 * A FAILED RUN LEAVES NO ZERO. If the seek times out or you abort it, the zero is
 * cleared rather than left at wherever the seek happened to start. A half-set
 * zero that other op modes inherit is the bug this whole exercise came from.
 *
 * CONTROLS (gamepad1)
 *   A              start the seek (or run it again)
 *   B              STOP. Aborts a seek and clears the zero.
 *   Left stick Y   jog by hand, up is up. Works any time, no limits.
 *   X              "it is level right now" -> prints LEVEL_ABOVE_STOP_DEG
 *   Y              forget the level capture
 *
 * Config: CR servo "deflector" + analog input "deflectorPos". Missing either and
 * it says so and does nothing.
 */
@Configurable
@TeleOp(name = "Deflector Auto Home", group = "Setup")
public class DeflectorAutoHome extends LinearOpMode {

    /** Down toward the stop. Gentle: this ends by pushing into a hard stop. */
    public static double SEEK_POWER = 0.25;

    /** -1 = down is negative power. Flip if the seek drives UP instead. */
    public static double SEEK_DIRECTION = -1.0;

    /** Ignore stall checks this long after starting, while it gets moving. */
    public static double SETTLE_SEC = 0.4;

    /** Movement under STALL_DEG across this window = sitting on the stop. */
    public static double STALL_WINDOW_SEC = 0.25;

    /** In DEFLECTOR degrees. 0.1 deg is about 3 servo degrees. */
    public static double STALL_DEG = 0.1;

    /** No stop found by now = give up, and leave no zero behind. */
    public static double TIMEOUT_SEC = 12.0;

    /** Hand-jog power at full stick. */
    public static double JOG_POWER = 0.3;

    private enum Phase { IDLE, SEEKING, FOUND, FAILED }

    private Phase phase = Phase.IDLE;
    private String note = "press A to seek the stop";

    private final ElapsedTime seekTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallRefAngle;
    private double lastWindowMovement = Double.NaN;

    /** Angle at the moment the seek started, so travel is readable on screen. */
    private double seekStartAngle;

    /** Captured with X: how far above the stop level is. */
    private double levelAboveStop = Double.NaN;

    @Override
    public void runOpMode() {
        Deflector deflector = new Deflector(hardwareMap);
        // An old zero may be wrong -- that is why we are here -- so its soft
        // limits must not wall off the seek or the hand jog.
        deflector.setJogIgnoresLimits(true);

        while (opModeInInit()) {
            deflector.update(false);   // track turns, move nothing
            if (!deflector.isPresent()) {
                telemetry.addLine("No deflector: needs CR servo \"" + Deflector.SERVO_NAME
                        + "\" and analog input \"" + Deflector.FEEDBACK_NAME + "\".");
            } else {
                telemetry.addLine("DEFLECTOR AUTO HOME -- nothing moves until you press A.");
                telemetry.addLine();
                telemetry.addData("Servo total", "%.0f deg", deflector.getServoTotal());
                telemetry.addLine("Turn it by hand and watch Servo total change, to");
                telemetry.addLine("prove the wire reads before trusting a seek.");
            }
            telemetry.update();
        }
        if (!deflector.isPresent()) return;

        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                abort(deflector, "stopped by B");
            }
            if (gamepad1.aWasPressed() && phase != Phase.SEEKING) {
                startSeek(deflector);
            }
            if (gamepad1.xWasPressed() && phase == Phase.FOUND) {
                levelAboveStop = deflector.getAngle();
            }
            if (gamepad1.yWasPressed()) {
                levelAboveStop = Double.NaN;
            }

            if (phase == Phase.SEEKING) {
                seek(deflector);
            } else {
                // Hand jog, any time outside a seek.
                double stick = -gamepad1.left_stick_y;
                deflector.jog(Math.abs(stick) > 0.05 ? stick * JOG_POWER : 0);
            }

            deflector.update(true);
            show(deflector);
            telemetry.update();
        }
        deflector.stop();
    }

    private void startSeek(Deflector deflector) {
        // Zero here first so the screen can show how far the seek has travelled.
        // The real zero gets set again the moment the stop is found.
        deflector.zeroHere();
        seekStartAngle = deflector.getAngle();
        stallRefAngle = seekStartAngle;
        lastWindowMovement = Double.NaN;
        levelAboveStop = Double.NaN;
        phase = Phase.SEEKING;
        note = "seeking the stop";
        seekTimer.reset();
        stallTimer.reset();
    }

    /** One loop of the seek. Drives down and decides whether it has stopped moving. */
    private void seek(Deflector deflector) {
        double angle = deflector.getAngle();

        if (seekTimer.seconds() > TIMEOUT_SEC) {
            abort(deflector, String.format(
                    "FAILED: no stop in %.0f s (moved %.1f deg). Wrong SEEK_DIRECTION, "
                            + "or the wire is not reading.", TIMEOUT_SEC, angle - seekStartAngle));
            return;
        }

        if (stallTimer.seconds() >= STALL_WINDOW_SEC) {
            double moved = Math.abs(angle - stallRefAngle);
            lastWindowMovement = moved;
            boolean settled = seekTimer.seconds() >= SETTLE_SEC;
            stallRefAngle = angle;
            stallTimer.reset();

            if (settled && moved < STALL_DEG) {
                // On the stop. THIS is the reference.
                deflector.jog(0);
                deflector.zeroHere();
                phase = Phase.FOUND;
                note = "STOP FOUND -- zero is here";
                return;
            }
        }

        deflector.jog(SEEK_DIRECTION * SEEK_POWER);
    }

    /**
     * Stop driving and leave NO zero behind. A zero set at wherever a failed seek
     * started would be inherited by every op mode after it, which is exactly how
     * the deflector's angles got silently wrong before.
     */
    private void abort(Deflector deflector, String why) {
        deflector.jog(0);
        deflector.stop();
        deflector.clearZero();
        phase = Phase.FAILED;
        note = why;
    }

    private void show(Deflector deflector) {
        telemetry.addData("Phase", "%s -- %s", phase, note);
        telemetry.addData("Zeroed", deflector.isZeroed() ? "yes" : "NO");
        telemetry.addLine();

        telemetry.addData("Angle", "%.2f deg", deflector.getAngle());
        telemetry.addData("Servo total", "%.0f deg  (unwrapped, counts turns)", deflector.getServoTotal());

        if (phase == Phase.SEEKING) {
            telemetry.addData("Seeking for", "%.1f s of %.0f", seekTimer.seconds(), TIMEOUT_SEC);
            telemetry.addData("Travelled", "%.1f deg", deflector.getAngle() - seekStartAngle);
            telemetry.addData("Last window moved", Double.isNaN(lastWindowMovement)
                    ? "(first window)"
                    : String.format("%.3f deg -- stop when under %.3f",
                            lastWindowMovement, STALL_DEG));
        }

        if (phase == Phase.FOUND) {
            telemetry.addLine();
            telemetry.addLine("Zero is the hard stop. Now jog UP to level and press X.");
            if (Double.isNaN(levelAboveStop)) {
                telemetry.addLine("Use a phone level on the deflector, not your eye.");
            } else {
                telemetry.addLine("==== WRITE THIS DOWN ====");
                telemetry.addData("LEVEL_ABOVE_STOP_DEG", "%.1f", levelAboveStop);
                telemetry.addData("  in servo degrees", "%.0f",
                        levelAboveStop * DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG);
                telemetry.addLine("Y clears it if you want to measure again.");
            }
        }

        telemetry.addLine();
        telemetry.addLine("A = seek   B = STOP   stick = jog   X = level here   Y = clear");
    }
}
