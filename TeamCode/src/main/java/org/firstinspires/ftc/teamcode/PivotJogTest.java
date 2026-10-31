package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Finds the pivot positions that matter, by hand, with no sensor involved.
 *
 * READ THIS FIRST, IT IS THE WHOLE REASON THE OP MODE WORKS THE WAY IT DOES
 *
 * Servo.getPosition() IS NOT A SENSOR. A standard servo has no feedback path to
 * the SDK -- getPosition() only reports the last value commanded through it, and
 * on a fresh op mode that is usually 0.0 no matter where the pivot physically
 * is. The first version of this op mode trusted it, believed the pivot was at
 * 0.0 while it was actually parked wide open, and the first bumper press
 * commanded 0.002 and slammed the servo across its entire travel into a hard
 * stop, where it hunted. That is fixed by not asking, and by never moving fast.
 *
 * TWO RULES THIS OP MODE FOLLOWS
 *
 * 1. NOTHING MOVES UNTIL YOU PRESS Y. Before that the servo is not written to at
 *    all, so you get to read the numbers and set START_POSITION first.
 *
 * 2. EVERY MOVE AFTER THAT IS RATE LIMITED. Jogs and presets both steer a
 *    TARGET; the commanded position creeps toward it at MAX_RATE per second. So
 *    a wrong number produces a slow traverse you can hear coming and abort by
 *    stopping the op mode, instead of a slam.
 *
 * THE ONE MOVE THAT CANNOT BE MADE SAFE
 * Pressing Y commands START_POSITION, and the servo goes there as fast as it
 * likes, because rate limiting can only smooth changes AFTER the first one.
 * Nothing in software can prevent that -- it cannot know where the pivot is, so
 * it cannot creep there from the current position.
 *
 * What it CAN do is land where the pivot already is. START_POSITION defaults to
 * 0.2, the open end measured on the robot 2026-10-30. Park the pivot fully open
 * before starting and pressing Y commands the position it is already in, so
 * nothing moves at all and the jump problem disappears.
 *
 * Parked somewhere else, Y still sends it to 0.2, which is known to be a legal
 * position. Change it in Panels if you want a different spot.
 *
 * Be ready on the stop button the first time, and listen. A servo pressed
 * against a stop buzzes.
 *
 * WHAT YOU ARE MEASURING
 *   OPEN END     wide open, for pulling a stuck ball out. MEASURED: 0.2
 *   CLOSED END   the tightest it goes before the servo strains
 *   POLLEN       where a pollen ball is just gripped
 *   NECTAR       where a nectar ball is just gripped
 *
 * The two ends matter most. ShooterPivot clips everything to that band, so once
 * they are recorded nothing can drive the pivot into a stop again.
 *
 * CONTROLS (gamepad1)
 *   Y                    take control. Commands START_POSITION. Press once.
 *   LB / RB              fine jog, one FINE_STEP per press
 *   L / R trigger        coarse jog, held
 *   X                    record OPEN end
 *   B                    record CLOSED end
 *   D-pad left / right   record POLLEN / NECTAR
 *   D-pad up / down      go to the recorded open / closed end
 *
 * HOW TO DO IT
 * 1. Park the pivot where you like, start the op mode, and look at the numbers.
 *    Nothing has moved yet.
 * 2. Press Y. Expect one jump to START_POSITION. If it buzzes, stop immediately
 *    and change START_POSITION.
 * 3. Jog toward one end until the servo starts to strain, back off two or three
 *    fine steps, and record that end. The straining sound IS the limit -- do not
 *    hold it there. Then find the other end.
 * 4. Then the grips, with a real ball between the flywheels.
 * 5. The constants to paste are printed at the bottom once both ends are in.
 *
 * WHICH DIRECTION IS "OPEN" IS NOT ASSUMED. ShooterPivot's fallbacks imply the
 * lower number is the wider gap; Jade's PieceDetector implies the opposite with
 * what look like untested placeholders. You jog, you look, you label it.
 */
@Configurable
@TeleOp(name = "Pivot Jog Test", group = "Test")
public class PivotJogTest extends LinearOpMode {

    /**
     * Where the pivot is commanded when you press Y. Defaults to the open end
     * measured 2026-10-30, so parking the pivot fully open makes Y a no-op.
     */
    public static double START_POSITION = 0.2;

    /** One bumper press. */
    public static double FINE_STEP = 0.002;

    /** Held-trigger jog, position units per second at full trigger. */
    public static double COARSE_RATE = 0.06;

    /**
     * How fast the commanded position is allowed to change, per second. Slow on
     * purpose: a full-range traverse takes about ten seconds, which is long
     * enough to hear a mistake and stop the op mode.
     */
    public static double MAX_RATE = 0.10;

    private double target;
    private double commanded;
    private boolean inControl = false;

    private double openEnd = Double.NaN;
    private double closedEnd = Double.NaN;
    private double pollenAt = Double.NaN;
    private double nectarAt = Double.NaN;

    private String note = "not in control - press Y when you have read the warning";

    private boolean lbPrev, rbPrev, xPrev, bPrev, yPrev;
    private boolean upPrev, downPrev, leftPrev, rightPrev;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Servo pivot = hardwareMap.tryGet(Servo.class, "shooterPivot");
        PanelsConfigurables.INSTANCE.refreshClass(this);

        if (pivot == null) {
            telemetry.addLine("No \"shooterPivot\" servo in the config - nothing to jog.");
            telemetry.addLine("Add it as a Servo on the Driver Station and re-run.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        target = START_POSITION;
        commanded = START_POSITION;

        telemetry.addLine("PIVOT JOG TEST");
        telemetry.addLine();
        telemetry.addLine("NOTHING WILL MOVE until you press Y.");
        telemetry.addData("Y will command", START_POSITION);
        telemetry.addLine();
        telemetry.addLine("The servo cannot be read, so that first move cannot be");
        telemetry.addLine("smoothed - it jumps. Park the pivot OPEN and it will not move.");
        telemetry.addLine("If it BUZZES, stop at once and change START_POSITION.");
        telemetry.addLine();
        telemetry.addLine("After that every move is rate limited and slow.");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            handleTakeControl();
            if (inControl) {
                handleJog(dt);
                handleRecord();
                handleGoTo();
                stepTowardTarget(dt);
                pivot.setPosition(commanded);
            }

            telemetry.addLine(inControl
                    ? "LB/RB fine | triggers coarse | X open end | B closed end"
                    : "PRESS Y TO TAKE CONTROL - nothing has moved yet");
            if (inControl) {
                telemetry.addLine("dpad L/R record pollen/nectar | dpad U/D go to ends");
            }
            telemetry.addLine();
            telemetry.addData("POSITION  <-- write this down", commanded);
            telemetry.addData("Target", target);
            telemetry.addData("Moving", Math.abs(target - commanded) > 1e-6 ? "yes" : "settled");
            telemetry.addLine();
            telemetry.addData("Open end   (X)", show(openEnd));
            telemetry.addData("Closed end (B)", show(closedEnd));
            telemetry.addData("Pollen (dpad L)", show(pollenAt));
            telemetry.addData("Nectar (dpad R)", show(nectarAt));
            telemetry.addLine();
            telemetry.addData("Travel band", bandDescription());
            telemetry.addData("Note", note);

            if (bothEndsKnown()) {
                telemetry.addLine();
                telemetry.addLine("Paste into ShooterPivot:");
                telemetry.addLine(String.format("  MIN_SAFE_POSITION = %.3f;", lowEnd()));
                telemetry.addLine(String.format("  MAX_SAFE_POSITION = %.3f;", highEnd()));
                if (!Double.isNaN(openEnd)) {
                    telemetry.addLine(String.format("  OPEN_POSITION = %.3f;", openEnd));
                }
                if (!Double.isNaN(pollenAt)) {
                    telemetry.addLine(String.format("  POLLEN_POSITION = %.3f;", pollenAt));
                }
                if (!Double.isNaN(nectarAt)) {
                    telemetry.addLine(String.format("  NECTAR_POSITION = %.3f;", nectarAt));
                }
            }
            telemetry.update();
        }
    }

    private void handleTakeControl() {
        boolean y = gamepad1.y;
        if (y && !yPrev && !inControl) {
            inControl = true;
            target = START_POSITION;
            commanded = START_POSITION;
            note = String.format("took control at %.3f", START_POSITION);
        }
        yPrev = y;
    }

    /** Jogging steers the TARGET. The servo follows it slowly. */
    private void handleJog(double dt) {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (rb && !rbPrev) {
            nudgeTarget(FINE_STEP);
        }
        if (lb && !lbPrev) {
            nudgeTarget(-FINE_STEP);
        }
        rbPrev = rb;
        lbPrev = lb;

        double coarse = (gamepad1.right_trigger - gamepad1.left_trigger) * COARSE_RATE * dt;
        if (coarse != 0.0) {
            nudgeTarget(coarse);
        }
    }

    private void nudgeTarget(double delta) {
        double wanted = target + delta;
        double clamped = clampToKnownBand(wanted);
        if (clamped != wanted) {
            note = "held at a recorded end - will not go past it";
        }
        target = clamped;
    }

    /** The rate limit. Nothing else writes `commanded`. */
    private void stepTowardTarget(double dt) {
        double maxStep = MAX_RATE * dt;
        double error = target - commanded;
        if (Math.abs(error) <= maxStep) {
            commanded = target;
        } else {
            commanded += Math.signum(error) * maxStep;
        }
    }

    /** Records where the pivot actually IS, not where it is heading. */
    private void handleRecord() {
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (x && !xPrev) {
            openEnd = commanded;
            target = commanded;
            note = String.format("OPEN end recorded at %.3f", commanded);
        }
        if (b && !bPrev) {
            closedEnd = commanded;
            target = commanded;
            note = String.format("CLOSED end recorded at %.3f", commanded);
        }
        if (left && !leftPrev) {
            pollenAt = commanded;
            note = String.format("POLLEN recorded at %.3f", commanded);
        }
        if (right && !rightPrev) {
            nectarAt = commanded;
            note = String.format("NECTAR recorded at %.3f", commanded);
        }

        xPrev = x;
        bPrev = b;
        leftPrev = left;
        rightPrev = right;
    }

    private void handleGoTo() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !upPrev) {
            goTo(openEnd, "open end");
        }
        if (down && !downPrev) {
            goTo(closedEnd, "closed end");
        }

        upPrev = up;
        downPrev = down;
    }

    private void goTo(double spot, String name) {
        if (Double.isNaN(spot)) {
            note = name + " has not been recorded yet";
            return;
        }
        target = clampToKnownBand(spot);
        note = "heading to " + name + " (rate limited)";
    }

    // ---- Travel band --------------------------------------------------------

    /**
     * Clamps ONLY once both ends are known.
     *
     * An earlier version tried to clamp against a single recorded end, and it
     * deadlocked the op mode on the robot 2026-10-30. Recording an end leaves
     * commanded == end, which satisfied the blocks in both directions at once
     * and pinned the pivot there with no way to jog off it.
     *
     * The real problem is that with one end recorded there is no way to know
     * WHICH SIDE of it is out of bounds, so the clamp was guessing. It does not
     * guess any more: jogging stays unrestricted until the second end is in,
     * which is why the instruction is to find them gently and listen.
     */
    private double clampToKnownBand(double wanted) {
        double result = Range.clip(wanted, 0.0, 1.0);
        if (bothEndsKnown()) {
            return Range.clip(result, lowEnd(), highEnd());
        }
        return result;
    }

    private boolean bothEndsKnown() {
        return !Double.isNaN(openEnd) && !Double.isNaN(closedEnd);
    }

    private double lowEnd() {
        return Math.min(openEnd, closedEnd);
    }

    private double highEnd() {
        return Math.max(openEnd, closedEnd);
    }

    private String bandDescription() {
        if (bothEndsKnown()) {
            return String.format("%.3f to %.3f (jogging limited to this)", lowEnd(), highEnd());
        }
        if (Double.isNaN(openEnd) && Double.isNaN(closedEnd)) {
            return "no ends recorded - full range allowed, jog gently";
        }
        return "one end recorded - still unrestricted until the other is in";
    }

    private String show(double value) {
        return Double.isNaN(value) ? "not recorded" : String.format("%.3f", value);
    }
}
