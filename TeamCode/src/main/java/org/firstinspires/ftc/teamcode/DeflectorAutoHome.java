package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

/**
 * Deflector Auto Home -- find HOME by itself and set ZERO from it. One job,
 * nothing else, so it can be proved before any of it gets wired into a match.
 *
 * TWO WORDS, AND THEY ARE NOT THE SAME PLACE
 *
 *   HOME  the hard stop at the bottom. Found by driving down until the angle
 *         stops changing. A physical fact, repeatable, needs no judgement.
 *   ZERO  parallel to the ground. The reference every angle in the code is
 *         measured from, so 70 means a 70 degree arc. It sits
 *         ZERO_ABOVE_HOME_DEG above HOME -- 11.9 on 6494, confirmed to land
 *         level consistently (Shane, 2026-11-10).
 *
 * Using HOME as the reference would be easier and wrong: the stop is not level,
 * so every angle would stop meaning an arc and none of the existing numbers
 * (DeflectorLogic.POSITIONS, RouteAuto.AUTO_ARC_DEG) would mean what they say.
 * HOME is how we FIND the mechanism; ZERO is what we MEASURE FROM.
 *
 * ZERO IS SET BY ARITHMETIC, NOT BY DRIVING TO IT
 * On HOME the op mode says "this position is -11.9", and level becomes 0 with
 * nothing having to move.
 *
 * The first version drove UP to level and zeroed there, and it failed every
 * time, stopping around 11.2 instead of 11.9. The position loop commands
 * kP * error floored at MIN_POWER = 0.05, and 5% will not lift this deflector
 * through its ~30:1. So it stalled about 0.7 deg short -- which is OUTSIDE the
 * 0.5 deg TOLERANCE_DEG, so it never reported arriving, the rise timed out, and
 * the abort path cleared the zero. A reference that depends on a mechanism
 * creeping uphill is the wrong design. Arithmetic cannot stall.
 *
 * THAT WEAKNESS IS STILL THERE FOR EVERY OTHER UPWARD MOVE, including auto's
 * goTo(AUTO_ARC_DEG) -- expect it to land ~0.7 deg low and sit there. Fixing it
 * means raising DeflectorLogic.MIN_POWER or adding gravity feedforward, which
 * touches every deflector move on both robots, so it is a separate decision.
 * The "Loop authority" readout below is the measurement for it.
 *
 * WHY THIS IS ITS OWN OP MODE
 * Auto-homing was switched on globally and added to Pollen Shot Tune in one
 * change (2026-11-10). When the zero came out wrong there was nothing to
 * isolate: Deflector Zero behaved differently, the tuner behaved differently,
 * and every angle downstream was suspect. That is reverted.
 *
 * IT TOUCHES NO SHARED STATE. It never calls Deflector.startHoming() and never
 * writes Deflector.LEVEL_ABOVE_STOP_DEG, which stays NaN on both robots -- so
 * autos and TeleOp keep refusing to auto-home exactly as they do today.
 *
 * IT PUSHES INTO HOME ON PURPOSE. That is what stall detection is. Low power, a
 * short window and a timeout keep it brief -- but watch it the first few runs,
 * and B stops everything.
 *
 * A FAILED OR ABORTED RUN LEAVES NO ZERO, rather than one at wherever the seek
 * happened to start. A half-set zero inherited by the next op mode through the
 * statics is the failure that started all this.
 *
 * CONTROLS (gamepad1)
 *   A              run it: seek HOME, set ZERO
 *   B              STOP. Aborts and clears the zero.
 *   Left stick Y   jog by hand, up is up. Any time, no limits.
 *   X              "this is true level" -> prints the corrected ZERO_ABOVE_HOME_DEG
 *   Y              forget that measurement
 *
 * Config: CR servo "deflector" + analog input "deflectorPos".
 */
@Configurable
@TeleOp(name = "Deflector Auto Home", group = "Setup")
public class DeflectorAutoHome extends LinearOpMode {

    /**
     * How far ZERO (parallel to the ground) sits above HOME (the hard stop), in
     * deflector degrees.
     *
     * 11.9 on 6494 (Shane, 2026-11-10), confirmed to land level consistently.
     * Deflector.LEVEL_ABOVE_STOP_DEG is still NaN on purpose: this moves there
     * once auto-homing itself is trusted, not just the number.
     *
     * WATCH OUT: 11.9 deflector degrees is 355 servo degrees -- 0.987 of a FULL
     * TURN, only 4.7 short of 360. Two consequences:
     *
     *   - the raw feedback at ZERO reads almost the same as at HOME, so voltage
     *     alone cannot tell them apart. Only the accumulated turn count can.
     *   - a dropped turn costs 12.06 deflector degrees, almost exactly the
     *     HOME-to-ZERO distance, so a lost turn makes ZERO look like HOME. A
     *     result about 12 deg out is the unwrap, not the stall detection --
     *     check "Max step" before suspecting anything else.
     */
    public static double ZERO_ABOVE_HOME_DEG = 11.9;

    /** Down toward HOME. Gentle: this ends by pushing into a hard stop. */
    public static double SEEK_POWER = 0.25;

    /** -1 = down is negative power. Flip if the seek drives UP instead. */
    public static double SEEK_DIRECTION = -1.0;

    /** Ignore stall checks this long after starting, while it gets moving. */
    public static double SETTLE_SEC = 0.4;

    /** Movement under STALL_DEG across this window = sitting on HOME. */
    public static double STALL_WINDOW_SEC = 0.25;

    /** In DEFLECTOR degrees. 0.1 deg is about 3 servo degrees. */
    public static double STALL_DEG = 0.1;

    /** No HOME found by now = give up, and leave no zero behind. */
    public static double SEEK_TIMEOUT_SEC = 12.0;

    /** Hand-jog power at full stick. */
    public static double JOG_POWER = 0.3;

    private enum Phase { IDLE, SEEKING_HOME, READY, FAILED }

    private Phase phase = Phase.IDLE;
    private String note = "press A to find HOME";

    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallRefAngle;
    private double lastWindowMovement = Double.NaN;
    private double seekStartAngle;

    /** Captured with X: how far true level is from where we called ZERO. */
    private double levelError = Double.NaN;

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
                telemetry.addData("ZERO is", "%.1f deg above HOME", ZERO_ABOVE_HOME_DEG);
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
            if (gamepad1.aWasPressed() && phase != Phase.SEEKING_HOME) {
                startSeek(deflector);
            }
            if (gamepad1.xWasPressed() && phase == Phase.READY) {
                levelError = deflector.getAngle();
            }
            if (gamepad1.yWasPressed()) {
                levelError = Double.NaN;
            }

            if (phase == Phase.SEEKING_HOME) {
                seekHome(deflector);
            } else {
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
        // The real ZERO is set the moment HOME is found.
        deflector.zeroHere();
        seekStartAngle = deflector.getAngle();
        stallRefAngle = seekStartAngle;
        lastWindowMovement = Double.NaN;
        levelError = Double.NaN;
        phase = Phase.SEEKING_HOME;
        note = "driving down to HOME";
        phaseTimer.reset();
        stallTimer.reset();
    }

    /** Drive down and decide when the angle has stopped responding: that is HOME. */
    private void seekHome(Deflector deflector) {
        double angle = deflector.getAngle();

        if (phaseTimer.seconds() > SEEK_TIMEOUT_SEC) {
            abort(deflector, String.format(
                    "FAILED: no HOME in %.0f s (moved %.1f deg). Wrong SEEK_DIRECTION, "
                            + "or the wire is not reading.",
                    SEEK_TIMEOUT_SEC, angle - seekStartAngle));
            return;
        }

        if (stallTimer.seconds() >= STALL_WINDOW_SEC) {
            double moved = Math.abs(angle - stallRefAngle);
            lastWindowMovement = moved;
            boolean settled = phaseTimer.seconds() >= SETTLE_SEC;
            stallRefAngle = angle;
            stallTimer.reset();

            if (settled && moved < STALL_DEG) {
                // On HOME. ZERO is set by arithmetic -- see the class comment for
                // why driving up to level instead failed at 11.2 every time.
                deflector.jog(0);
                deflector.zeroAt(-ZERO_ABOVE_HOME_DEG);
                phase = Phase.READY;
                note = "HOME found, ZERO set -- nothing had to move";
                return;
            }
        }

        deflector.jog(SEEK_DIRECTION * SEEK_POWER);
    }

    /**
     * Stop driving and leave NO zero behind. A zero set at wherever a failed run
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

        double angle = deflector.getAngle();
        telemetry.addData("Angle", "%.2f deg  %s", angle,
                phase == Phase.READY ? "from ZERO (level)" : "from where the run started");
        if (phase == Phase.READY) {
            telemetry.addData("HOME is at", "%.1f deg", -ZERO_ABOVE_HOME_DEG);
        }
        telemetry.addData("ZERO is", "%.1f deg above HOME", ZERO_ABOVE_HOME_DEG);
        telemetry.addData("Servo total", "%.0f deg  (unwrapped, counts turns)",
                deflector.getServoTotal());

        // The unwrap assumes under 180 servo deg per loop. HOME-to-ZERO is 0.987
        // of a turn, so a dropped turn lands almost exactly on the other one --
        // this says whether that is close to happening.
        telemetry.addData("Max step", "%.0f servo deg/loop%s", deflector.getMaxStepSeen(),
                deflector.getMaxStepSeen() > 120 ? "   <-- NEAR THE 180 UNWRAP LIMIT"
                        : "   (unwrap safe under 180)");

        // What the position loop would command right here, heading for level.
        // Under about 0.06 it cannot lift the deflector, which is why upward
        // goTo moves stall short. This is the measurement for raising MIN_POWER.
        telemetry.addData("Loop authority", "%.3f power at %.2f deg of error",
                Math.abs(DeflectorLogic.power(angle, 0)), Math.abs(angle));

        if (phase == Phase.SEEKING_HOME) {
            telemetry.addLine();
            telemetry.addData("Seeking for", "%.1f s of %.0f",
                    phaseTimer.seconds(), SEEK_TIMEOUT_SEC);
            telemetry.addData("Travelled", "%.1f deg", angle - seekStartAngle);
            telemetry.addData("Last window moved", Double.isNaN(lastWindowMovement)
                    ? "(first window)"
                    : String.format("%.3f deg -- HOME when under %.3f",
                            lastWindowMovement, STALL_DEG));
        }

        if (phase == Phase.READY) {
            telemetry.addLine();
            telemetry.addLine("Jog to true level and press X to re-check ZERO_ABOVE_HOME_DEG.");
            if (!Double.isNaN(levelError)) {
                telemetry.addLine("==== ZERO_ABOVE_HOME_DEG ====");
                telemetry.addData("Corrected", "%.1f   (is %.1f, off by %+.1f)",
                        ZERO_ABOVE_HOME_DEG + levelError, ZERO_ABOVE_HOME_DEG, levelError);
            }
        }

        telemetry.addLine();
        telemetry.addLine("A = run   B = STOP   stick = jog   X = level here   Y = clear");
    }
}
