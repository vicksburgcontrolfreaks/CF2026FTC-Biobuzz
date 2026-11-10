package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooting.DeflectorLogic;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

/**
 * Deflector Auto Home -- find HOME by itself, set ZERO from it, and measure how
 * far apart they really are. One job, nothing else, so it can be proved before
 * any of it gets wired into a match path.
 *
 * TWO WORDS, AND THEY ARE NOT THE SAME PLACE
 *
 *   HOME  the hard stop at the bottom. Found by driving down until the angle
 *         stops changing. A physical fact, repeatable, needs no judgement.
 *   ZERO  parallel to the ground. This is the reference every angle in the code
 *         is measured from, so 70 means a 70 degree arc. It is
 *         ZERO_ABOVE_HOME_DEG above HOME -- currently ESTIMATED at 15.
 *
 * Using HOME as the reference would be easier and wrong: the stop is not level,
 * so every angle would stop meaning an arc and none of the existing numbers
 * (DeflectorLogic.POSITIONS, RouteAuto.AUTO_ARC_DEG) would mean what they say.
 * So HOME is how we FIND the mechanism, and ZERO is what we MEASURE FROM.
 *
 * THE FINE-TUNE LOOP, which is the point of this op mode
 *   1. A -- it seeks HOME, then rises ZERO_ABOVE_HOME_DEG and calls that ZERO.
 *   2. Look at the deflector. Is it actually parallel to the ground? Use a phone
 *      level, not your eye.
 *   3. If not, jog to true level and press X. It prints the corrected
 *      ZERO_ABOVE_HOME_DEG -- the estimate plus however far off it was.
 *   4. Put that number in ZERO_ABOVE_HOME_DEG and run it again. Repeat until
 *      step 2 says yes, and the deflector zeroes itself correctly every time
 *      with nobody eyeballing anything.
 *
 * WHY THIS IS ITS OWN OP MODE
 * Auto-homing was switched on globally and added to Pollen Shot Tune in one
 * change (2026-11-10). When the zero came out wrong there was nothing to
 * isolate: Deflector Zero behaved differently, the tuner's zeroing behaved
 * differently, and every angle downstream was suspect. That is reverted.
 *
 * IT TOUCHES NO SHARED STATE. It never calls Deflector.startHoming() and never
 * writes Deflector.LEVEL_ABOVE_STOP_DEG, which stays NaN on both robots -- so
 * autos and TeleOp keep refusing to auto-home exactly as they do today. The seek
 * lives in this file, in constants you can read. Running this op mode cannot
 * change how anything else behaves.
 *
 * IT PUSHES INTO HOME ON PURPOSE. That is what stall detection is. Low power, a
 * short window and a timeout keep it brief -- but watch the mechanism the first
 * few runs, and B stops everything.
 *
 * A FAILED OR ABORTED RUN LEAVES NO ZERO, rather than one at wherever the seek
 * happened to start. A half-set zero inherited by the next op mode through the
 * statics is the failure that started all this.
 *
 * CONTROLS (gamepad1)
 *   A              run it: seek HOME, rise to ZERO
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
     * 15 is an ESTIMATE (Shane, 2026-11-10), not a measurement. Refining it is
     * what this op mode is for -- see the fine-tune loop above. Once it is right,
     * this is the number that goes into Deflector.LEVEL_ABOVE_STOP_DEG and makes
     * auto-homing trustworthy everywhere else.
     */
    public static double ZERO_ABOVE_HOME_DEG = 15.0;

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

    /** HOME -> ZERO is only ~1.2 turns, so this is generous. */
    public static double RISE_TIMEOUT_SEC = 6.0;

    /** Hand-jog power at full stick. */
    public static double JOG_POWER = 0.3;

    private enum Phase { IDLE, SEEKING_HOME, RISING_TO_ZERO, READY, FAILED }

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
                telemetry.addData("ZERO is estimated", "%.1f deg above HOME", ZERO_ABOVE_HOME_DEG);
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
            if (gamepad1.aWasPressed()
                    && phase != Phase.SEEKING_HOME && phase != Phase.RISING_TO_ZERO) {
                startSeek(deflector);
            }
            if (gamepad1.xWasPressed() && phase == Phase.READY) {
                levelError = deflector.getAngle();
            }
            if (gamepad1.yWasPressed()) {
                levelError = Double.NaN;
            }

            switch (phase) {
                case SEEKING_HOME:   seekHome(deflector); break;
                case RISING_TO_ZERO: riseToZero(deflector); break;
                default:
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
        // HOME becomes the zero when it is found, and ZERO replaces that after
        // the rise -- this first one is only to make the readout meaningful.
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
                            + "or the wire is not reading.", SEEK_TIMEOUT_SEC, angle - seekStartAngle));
            return;
        }

        if (stallTimer.seconds() >= STALL_WINDOW_SEC) {
            double moved = Math.abs(angle - stallRefAngle);
            lastWindowMovement = moved;
            boolean settled = phaseTimer.seconds() >= SETTLE_SEC;
            stallRefAngle = angle;
            stallTimer.reset();

            if (settled && moved < STALL_DEG) {
                // On HOME. Call it 0 for now, so the rise can be commanded as a
                // plain "go up ZERO_ABOVE_HOME_DEG".
                deflector.jog(0);
                deflector.zeroHere();
                deflector.goTo(ZERO_ABOVE_HOME_DEG);
                phase = Phase.RISING_TO_ZERO;
                note = String.format("HOME found -- rising %.1f deg to ZERO", ZERO_ABOVE_HOME_DEG);
                phaseTimer.reset();
                return;
            }
        }

        deflector.jog(SEEK_DIRECTION * SEEK_POWER);
    }

    /** Ride the position loop up to the estimated ZERO, then declare it zero. */
    private void riseToZero(Deflector deflector) {
        boolean arrived = Math.abs(deflector.getAngle() - ZERO_ABOVE_HOME_DEG)
                <= DeflectorLogic.TOLERANCE_DEG;

        if (arrived) {
            deflector.zeroHere();   // this position is now 0 = level, by definition
            phase = Phase.READY;
            note = "ZERO set. Check it is actually level.";
            return;
        }
        if (phaseTimer.seconds() > RISE_TIMEOUT_SEC) {
            abort(deflector, String.format(
                    "FAILED: did not reach ZERO in %.0f s (got %.1f of %.1f).",
                    RISE_TIMEOUT_SEC, deflector.getAngle(), ZERO_ABOVE_HOME_DEG));
        }
        // Otherwise the subsystem's position loop is already driving it.
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

        String frame = phase == Phase.READY ? "from ZERO (level)"
                : phase == Phase.RISING_TO_ZERO ? "from HOME"
                : "from where the run started";
        telemetry.addData("Angle", "%.2f deg  %s", deflector.getAngle(), frame);
        telemetry.addData("Servo total", "%.0f deg  (unwrapped, counts turns)",
                deflector.getServoTotal());
        telemetry.addData("ZERO estimate", "%.1f deg above HOME", ZERO_ABOVE_HOME_DEG);

        if (phase == Phase.SEEKING_HOME) {
            telemetry.addLine();
            telemetry.addData("Seeking for", "%.1f s of %.0f", phaseTimer.seconds(), SEEK_TIMEOUT_SEC);
            telemetry.addData("Travelled", "%.1f deg", deflector.getAngle() - seekStartAngle);
            telemetry.addData("Last window moved", Double.isNaN(lastWindowMovement)
                    ? "(first window)"
                    : String.format("%.3f deg -- HOME when under %.3f",
                            lastWindowMovement, STALL_DEG));
        }

        if (phase == Phase.READY) {
            telemetry.addLine();
            telemetry.addLine("Is the deflector parallel to the ground? Phone level, not your eye.");
            if (Double.isNaN(levelError)) {
                telemetry.addLine("If not: jog to true level, then press X.");
            } else {
                telemetry.addLine("==== PUT THIS IN ZERO_ABOVE_HOME_DEG ====");
                telemetry.addData("Corrected", "%.1f   (was %.1f, off by %+.1f)",
                        ZERO_ABOVE_HOME_DEG + levelError, ZERO_ABOVE_HOME_DEG, levelError);
                telemetry.addData("  in servo degrees", "%.0f",
                        (ZERO_ABOVE_HOME_DEG + levelError)
                                * DeflectorLogic.SERVO_DEG_PER_DEFLECTOR_DEG);
                telemetry.addLine("Then run A again and check level once more.");
            }
        }

        telemetry.addLine();
        telemetry.addLine("A = run   B = STOP   stick = jog   X = level here   Y = clear");
    }
}
