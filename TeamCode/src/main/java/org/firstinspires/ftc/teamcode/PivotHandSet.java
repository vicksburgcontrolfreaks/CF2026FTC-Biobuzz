package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Pivot Hand Set: set the shooter pivot BY HAND, and get the servo number for it.
 *
 * The pivot servo goes limp (signal off), you push the mechanism where you want
 * it, and the "pivotPos" feedback wire (EH analog 3) tells us where that is.
 * The feedback wire reads the servo's real shaft angle, powered or not.
 *
 * Feedback is a VOLTAGE, but the code commands the servo with a 0-1 POSITION.
 * Those are different scales, so step 3 powers the servo and walks it until
 * the feedback voltage matches what you set by hand. The command it lands on
 * is the number for ShooterPivot (POLLEN_POSITION for this robot).
 *
 * CONTROLS (gamepad1)
 *   A   record the HARD STOP (push the pivot against it by hand first)
 *   B   record the HAND-SET spot (push it where you want it first)
 *   Y   power the servo and find the command for the hand-set spot. HANDS OFF.
 *   X   go limp again (servo off), any time
 *   D-pad up  HOLD at this robot's saved POLLEN_POSITION, right away, no
 *             feedback needed. For re-attaching a slipped horn/linkage: with
 *             the linkage DISCONNECTED, hold here, set the mechanism to the
 *             pollen gap by hand, and re-tighten. Every saved number then
 *             lines up again. Do NOT use with a slipped linkage still
 *             attached -- the saved number may be past a hard stop.
 *
 * Works in INIT too, so you can do steps A and B before pressing Start.
 * Y only works after Start.
 *
 * NOT ROBOT-TESTED YET (2026-11-12). Things to watch the first time:
 *   - "Live" voltage should change smoothly as you move the pivot by hand. If it
 *     doesn't move, pivotPos is the wrong wire or isn't configured.
 *   - On Y, the servo jumps to the middle of its safe band first (the same kind
 *     of jump Pollen Shot Tune makes at Start). Be ready on stop if it buzzes.
 */
@TeleOp(name = "Pivot Hand Set", group = "Test")
public class PivotHandSet extends LinearOpMode {

    private static final String SERVO_NAME = "shooterPivot";
    private static final String FEEDBACK_NAME = "pivotPos";

    /** Axon feedback: about 0 V at 0 deg to about 3.3 V at 360 deg. */
    private static final double FULL_TURN_DEG = 360.0;

    /** Close enough, in volts. 0.01 V is about 1 degree. */
    public static double TOLERANCE_V = 0.01;
    /** How far to step for the slope measurement, in servo position units. */
    public static double PROBE_STEP = 0.05;
    /** Max speed the command may change, position units per second. Slow on purpose. */
    public static double MAX_RATE = 0.10;
    /** Wait after a move for the servo to settle before reading feedback. */
    public static long SETTLE_MS = 500;
    public static int MAX_TRIES = 8;

    private ServoImplEx servo;
    private AnalogInput feedback;
    private ShooterPivot pivotNumbers;   // only used for this robot's safe band

    private double stopV = Double.NaN;
    private double handV = Double.NaN;
    private double commanded = Double.NaN;   // NaN = we have not commanded anything yet
    private double foundCommand = Double.NaN;
    private double stopCommandEstimate = Double.NaN;
    private boolean limp = true;
    private String note = "Servo is OFF. Move the pivot by hand.";

    private boolean aPrev, bPrev, xPrev, yPrev, upPrev;

    @Override
    public void runOpMode() {
        servo = hardwareMap.tryGet(ServoImplEx.class, SERVO_NAME);
        feedback = hardwareMap.tryGet(AnalogInput.class, FEEDBACK_NAME);
        if (servo == null || feedback == null) {
            telemetry.addLine(">>> Missing from the robot config:");
            if (servo == null) telemetry.addLine("    servo \"" + SERVO_NAME + "\"");
            if (feedback == null) telemetry.addLine("    analog input \"" + FEEDBACK_NAME + "\"");
            telemetry.update();
            waitForStart();
            return;
        }
        // Constructing it loads this robot's safe band; it does not move the servo.
        pivotNumbers = new ShooterPivot(servo);
        goLimp();

        while (opModeInInit()) {
            handleButtons(false);
            show();
        }
        while (opModeIsActive()) {
            handleButtons(true);
            show();
        }
        goLimp();
    }

    private void handleButtons(boolean started) {
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
        boolean up = gamepad1.dpad_up;

        if (up && !upPrev) {
            if (!started) {
                note = "Press Start first -- the servo can't move during INIT.";
            } else {
                limp = false;
                commanded = ShooterPivot.POLLEN_POSITION;
                servo.setPosition(commanded);
                note = String.format("HOLDING saved pollen %.3f. Set the mechanism to the pollen "
                        + "gap by hand and re-tighten the linkage. X = servo off.", commanded);
            }
        }
        upPrev = up;

        if (x && !xPrev) {
            goLimp();
            note = "Servo is OFF. Move the pivot by hand.";
        }
        if (a && !aPrev) {
            if (!limp) {
                note = "Press X first (servo off) before setting by hand.";
            } else {
                stopV = readVolts();
                note = "Hard stop recorded. Now move it to the spot you want and press B.";
            }
        }
        if (b && !bPrev) {
            if (!limp) {
                note = "Press X first (servo off) before setting by hand.";
            } else {
                handV = readVolts();
                note = started ? "Hand-set spot recorded. HANDS OFF, then press Y."
                               : "Hand-set spot recorded. Press Start, then Y (hands off).";
            }
        }
        if (y && !yPrev) {
            if (!started) {
                note = "Press Start first -- the servo can't move during INIT.";
            } else if (Double.isNaN(handV)) {
                note = "Record the hand-set spot (B) first.";
            } else {
                findCommand();
            }
        }
        aPrev = a; bPrev = b; xPrev = x; yPrev = y;
    }

    /**
     * Walks the servo command until the feedback voltage matches handV.
     * 1. Go to mid-band, read the voltage.
     * 2. Step PROBE_STEP, read again -> how many volts per position unit.
     * 3. Use that to guess the command, go there, correct, repeat.
     */
    private void findCommand() {
        foundCommand = Double.NaN;
        stopCommandEstimate = Double.NaN;
        double min = ShooterPivot.MIN_SAFE_POSITION;
        double max = ShooterPivot.MAX_SAFE_POSITION;
        double mid = (min + max) / 2.0;

        // First command: there is no "current" command to creep from, so this
        // one jumps. Mid-band is the move most likely to be safe.
        limp = false;
        commanded = mid;
        servo.setPosition(commanded);   // also turns the servo signal back on
        if (!settle()) return;
        double v1 = readVolts();

        double c2 = Range.clip(mid + PROBE_STEP, min, max);
        if (!creepTo(c2)) return;
        double v2 = readVolts();
        if (Math.abs(v2 - v1) < 0.02) {
            note = String.format("Feedback barely moved (%.3f -> %.3f V) when the servo moved. "
                    + "Is pivotPos the right wire? Press X.", v1, v2);
            return;
        }
        double posPerVolt = (c2 - mid) / (v2 - v1);

        double v = v2;
        for (int i = 0; i < MAX_TRIES; i++) {
            double err = handV - v;
            if (Math.abs(err) <= TOLERANCE_V) {
                foundCommand = commanded;
                if (!Double.isNaN(stopV)) {
                    stopCommandEstimate = commanded + (stopV - handV) * posPerVolt;
                }
                note = "FOUND IT. Servo is holding the hand-set spot. Check it by eye.";
                return;
            }
            double next = Range.clip(commanded + err * posPerVolt, min, max);
            if (next == commanded) {
                note = String.format("Hand-set spot is OUTSIDE the safe band (%.3f - %.3f) for %s. "
                        + "Stopped at the edge, %.3f V short. The band needs re-measuring. Press X.",
                        min, max, WhichRobot.number(), err);
                return;
            }
            if (!creepTo(next)) return;
            v = readVolts();
        }
        note = String.format("Didn't settle within %d tries (%.3f V off). Servo may be "
                + "fighting slop. Press Y to try again.", MAX_TRIES, handV - v);
    }

    /** Rate-limited move, so a bad number is a slow traverse, not a slam. */
    private boolean creepTo(double target) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && commanded != target) {
            if (gamepad1.x) { goLimp(); note = "Stopped (X). Servo is OFF."; return false; }
            double maxStep = MAX_RATE * t.seconds();
            t.reset();
            commanded += Range.clip(target - commanded, -maxStep, maxStep);
            servo.setPosition(commanded);
            show();
        }
        return settle();
    }

    private boolean settle() {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < SETTLE_MS) {
            if (gamepad1.x) { goLimp(); note = "Stopped (X). Servo is OFF."; return false; }
            show();
        }
        return opModeIsActive();
    }

    private void goLimp() {
        servo.setPwmDisable();
        limp = true;
    }

    /** Averaged, because a single analog read is a little noisy. */
    private double readVolts() {
        double sum = 0;
        int n = 20;
        for (int i = 0; i < n; i++) sum += feedback.getVoltage();
        return sum / n;
    }

    private double toDeg(double volts) {
        return volts / feedback.getMaxVoltage() * FULL_TURN_DEG;
    }

    private void show() {
        double live = feedback.getVoltage();
        telemetry.addLine("PIVOT HAND SET  (" + WhichRobot.number() + ")");
        telemetry.addData("Servo", limp ? "OFF (limp) -- move by hand"
                : String.format("ON, commanded %.3f", commanded));
        telemetry.addData("Live", "%.3f V  (%.1f deg)", live, toDeg(live));
        telemetry.addLine();
        telemetry.addData("A  Hard stop", Double.isNaN(stopV) ? "--"
                : String.format("%.3f V  (%.1f deg)", stopV, toDeg(stopV)));
        telemetry.addData("B  Hand-set", Double.isNaN(handV) ? "--"
                : String.format("%.3f V  (%.1f deg)", handV, toDeg(handV)));
        if (!Double.isNaN(stopV) && !Double.isNaN(handV)) {
            telemetry.addData("   Moved from stop", "%+.3f V  (%+.1f deg)",
                    handV - stopV, toDeg(handV) - toDeg(stopV));
        }
        if (!Double.isNaN(stopV) && limp) {
            telemetry.addData("   Live from stop", "%+.1f deg", toDeg(live) - toDeg(stopV));
        }
        telemetry.addLine();
        if (!Double.isNaN(foundCommand)) {
            telemetry.addLine(String.format(">>> SERVO NUMBER: %.3f", foundCommand));
            telemetry.addLine("    -> POLLEN_POSITION for " + WhichRobot.number()
                    + " in ShooterPivot.loadRobotNumbers()");
            if (!Double.isNaN(stopCommandEstimate)) {
                telemetry.addLine(String.format("    (hard stop is about %.3f -- estimate, "
                        + "do NOT command it)", stopCommandEstimate));
            }
        }
        telemetry.addData("Safe band", "%.3f - %.3f",
                ShooterPivot.MIN_SAFE_POSITION, ShooterPivot.MAX_SAFE_POSITION);
        telemetry.addLine(note);
        telemetry.addLine("A=stop  B=hand-set  Y=find number  X=servo off");
        telemetry.addLine(String.format("D-pad up = hold saved pollen %.3f (linkage OFF!)",
                ShooterPivot.POLLEN_POSITION));
        telemetry.update();
    }
}
