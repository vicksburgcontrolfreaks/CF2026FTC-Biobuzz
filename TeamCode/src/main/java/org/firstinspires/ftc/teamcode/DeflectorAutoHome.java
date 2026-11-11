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
 * (DeflectorLogic.POSITIONS, Deflector.DEFAULT_SHOT_DEG) would mean what they say.
 * HOME is how we FIND the mechanism; ZERO is what we MEASURE FROM.
 *
 * ZERO IS SET BY ARITHMETIC; THE CLIMB IS SEPARATE
 * On HOME the op mode says "this position is -11.9", so level becomes 0 with
 * nothing having to move. THEN it climbs to RISE_TO_DEG -- the stow position,
 * 35 deg above ZERO -- and stops there, ready for auton or TeleOp. Two separate things: the reference cannot fail, and the climb is a
 * plain move that either arrives or says it did not.
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
 * goTo(DEFAULT_SHOT_DEG) -- expect it to land ~0.7 deg low and sit there. Fixing it
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

    /** A stall only means HOME if the seek travelled at least this far first. */
    public static double MIN_TRAVEL_DEG = 2.0;

    /** No HOME found by now = give up, and leave no zero behind. */
    public static double SEEK_TIMEOUT_SEC = 12.0;

    // ---- Rising from HOME to ZERO -------------------------------------------
    // Done with jog power, NOT the position loop. The loop commands kP * error
    // floored at MIN_POWER = 0.05, and 5% will not lift this deflector, so it
    // stalls short and at a different place every time. These are plain powers
    // that do move it, with a slow final approach so it does not coast past.

    /**
     * Where the run finishes: the STOW position, in degrees above ZERO.
     *
     * 35 (Shane, 2026-11-10) -- close to the shooting angles so the first move
     * in a match is short, while still inside the 18 in. starting cube. The
     * full park is therefore HOME + ZERO_ABOVE_HOME_DEG + 35.
     *
     * This is the number that becomes the subsystem.s stow angle once auton and
     * TeleOp pick the routine up.
     */
    public static double RISE_TO_DEG = 35.0;

    /** Fast part of the climb. */
    public static double RISE_POWER = 0.25;

    /** Within this much of RISE_TO_DEG, slow down. */
    public static double RISE_SLOW_DEG = 2.0;

    /** Slow part. Must still be enough to actually lift it. */
    public static double RISE_SLOW_POWER = 0.12;

    // 8.0 was sized for the old climb to level, which was 11.9 deg -- about ONE
    // servo turn. Climbing to 35 is 46.9 deg from HOME, 1400 servo deg, nearly
    // FOUR turns, so it takes roughly four times as long and 8 s would have
    // timed out mid-climb and cleared the zero. INIT can be held as long as
    // anyone likes, so this errs long on purpose.
    public static double RISE_TIMEOUT_SEC = 20.0;

    /** Hand-jog power at full stick. */
    public static double JOG_POWER = 0.3;

    private enum Phase { IDLE, SEEKING_HOME, RISING, READY, FAILED }

    private Phase phase = Phase.IDLE;
    private String note = "press A to find HOME";

    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallRefAngle;
    private double lastWindowMovement = Double.NaN;
    private double seekStartAngle;

    /** Where the climb cut power. Compared with the live angle to show drift. */
    private double stoppedAtAngle = Double.NaN;

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
            } else if (phase == Phase.RISING) {
                rise(deflector);
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
        stoppedAtAngle = Double.NaN;
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

            // Not moving AND never moved = blocked or not driven, NOT the stop.
            // Calling that HOME is what mis-pointed the deflector in auto on
            // 2026-11-10: the zero got set at the stow and the climb went 46.9
            // deg past it, twice.
            double travelled = Math.abs(angle - seekStartAngle);
            if (settled && moved < STALL_DEG && travelled < MIN_TRAVEL_DEG) {
                abort(deflector, String.format(
                        "FAILED: never moved (%.2f of %.2f deg). Blocked, or the servo "
                                + "is not driving. Zero NOT set.", travelled, MIN_TRAVEL_DEG));
                return;
            }

            if (settled && moved < STALL_DEG) {
                // On HOME. ZERO is set by arithmetic -- see the class comment for
                // why driving up to level instead failed at 11.2 every time.
                deflector.jog(0);
                deflector.zeroAt(-ZERO_ABOVE_HOME_DEG);
                phase = Phase.RISING;
                note = String.format("HOME found, ZERO set -- climbing to %.1f", RISE_TO_DEG);
                phaseTimer.reset();
                return;
            }
        }

        deflector.jog(SEEK_DIRECTION * SEEK_POWER);
    }

    /**
     * Climb from HOME to RISE_TO_DEG on JOG POWER, not the position loop.
     *
     * The loop cannot do this: it commands kP * error floored at MIN_POWER =
     * 0.05, and 5% will not lift this deflector, so it stalls short and in a
     * different place every run. Jogging at a power that actually moves it, then
     * easing off for the last RISE_SLOW_DEG, lands on the target instead of near
     * it. Releasing the jog makes the subsystem hold wherever it stopped.
     */
    private void rise(Deflector deflector) {
        double angle = deflector.getAngle();
        double remaining = RISE_TO_DEG - angle;

        if (remaining <= 0) {
            deflector.jog(0);   // Deflector.jog(0) holds where it stopped
            stoppedAtAngle = angle;
            phase = Phase.READY;
            note = "STOW reached";   // no number here: the readout below is live
            return;
        }
        if (phaseTimer.seconds() > RISE_TIMEOUT_SEC) {
            abort(deflector, String.format(
                    "FAILED: stopped climbing %.2f deg short of %.1f. RISE_SLOW_POWER "
                            + "may be too weak to lift it.", remaining, RISE_TO_DEG));
            return;
        }
        deflector.jog(remaining > RISE_SLOW_DEG ? RISE_POWER : RISE_SLOW_POWER);
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
                phase == Phase.IDLE || phase == Phase.SEEKING_HOME
                        ? "from where the run started"
                        : "from ZERO (level)");
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

        // What the position loop would command to hold the stow from here. Under
        // about 0.06 it cannot lift the deflector, which is why it sags a few
        // tenths after the climb and the loop cannot pull it back. This is the
        // measurement for raising MIN_POWER.
        telemetry.addData("Loop authority", "%.3f power at %.2f deg of error",
                Math.abs(DeflectorLogic.power(angle, RISE_TO_DEG)),
                Math.abs(RISE_TO_DEG - angle));

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

        if (phase == Phase.RISING) {
            telemetry.addLine();
            telemetry.addData("Climbing to", "%.1f deg -- %.2f to go", RISE_TO_DEG,
                    RISE_TO_DEG - angle);
            telemetry.addData("Jog power", "%.2f%s",
                    Math.abs(RISE_TO_DEG - angle) > RISE_SLOW_DEG ? RISE_POWER : RISE_SLOW_POWER,
                    Math.abs(RISE_TO_DEG - angle) > RISE_SLOW_DEG ? " (fast)" : " (slow approach)");
        }

        if (phase == Phase.READY) {
            telemetry.addLine();
            // Two different numbers, which was confusing before: where the climb
            // CUT POWER, and where it is NOW. The gap is what the mechanism did
            // afterwards -- coasting up or sagging back down -- and the hold loop
            // cannot correct it, because at a few tenths of error it commands
            // about MIN_POWER and that will not lift this deflector. So this gap
            // IS the accuracy of the whole routine right now.
            telemetry.addData("Stopped at", "%.2f deg  (when power was cut)", stoppedAtAngle);
            telemetry.addData("Resting at", "%.2f deg  (live)", angle);
            telemetry.addData("Drifted", "%+.2f deg after power off%s", angle - stoppedAtAngle,
                    Math.abs(angle - stoppedAtAngle) > DeflectorLogic.TOLERANCE_DEG
                            ? "   -- beyond tolerance, the hold loop is too weak to pull it back"
                            : "");
            telemetry.addLine("X still re-checks ZERO_ABOVE_HOME_DEG, but only from LEVEL --");
            telemetry.addLine("jog down to level first, or the number will be meaningless.");
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
