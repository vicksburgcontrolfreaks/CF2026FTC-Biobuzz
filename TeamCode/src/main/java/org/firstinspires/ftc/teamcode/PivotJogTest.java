package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Finds the four pivot positions that matter, by hand, with no sensor involved.
 *
 * THE BEAM BREAK IS DELIBERATELY NOT USED. It has no power, and a standard servo
 * does not need homing anyway -- setPosition() is absolute, so the servo goes to
 * the same place every boot. The beam was only ever there to detect the
 * mechanism shifting under the servo (a slipped horn, a popped linkage), which
 * is a rarer problem and not worth blocking on. This op mode needs
 * "shooterPivot" and nothing else.
 *
 * WHAT YOU ARE MEASURING
 *   OPEN END     wide open, where a stuck ball can be pulled out
 *   CLOSED END   the tightest it goes before the servo strains
 *   POLLEN       where a pollen ball is just gripped
 *   NECTAR       where a nectar ball is just gripped
 *
 * The two ends are the important ones. ShooterPivot clips everything to that
 * band, so once they are recorded nothing can drive the pivot into a hard stop
 * -- which is what hunted the servo on 2026-10-30.
 *
 * CONTROLS (gamepad1)
 *
 *   JOG
 *     LB / RB            fine jog, one FINE_STEP per press
 *     L / R trigger      coarse jog, held. Rate-based, so it is steerable
 *
 *   RECORD (face buttons) -- records wherever the pivot is right now
 *     X                  this is the OPEN end
 *     B                  this is the CLOSED end
 *     Y                  this is POLLEN grip
 *     A                  this is NECTAR grip
 *
 *   GO TO (d-pad) -- only works once that spot has been recorded
 *     up                 open        down     closed end
 *     left               pollen      right    nectar
 *
 * HOW TO DO IT
 *
 * 1. LOOK BEFORE YOU TOUCH. "Servo reports" is the position the servo thinks it
 *    is at. Nothing is commanded until your first jog, on purpose -- so if that
 *    number looks wrong, fine-jog ONE step first and watch which way the pivot
 *    actually moves before doing anything else.
 *
 * 2. FIND THE ENDS FIRST, GENTLY. Jog toward one end until you hear the servo
 *    start to strain, then back off two or three fine steps and record that as
 *    the end. The straining sound IS the limit -- do not hold it there. Repeat
 *    for the other end. Record both before you go looking for grip positions,
 *    because after that the op mode will not let you leave the band.
 *
 * 3. THEN THE GRIPS. Put a pollen ball between the flywheels, jog until it is
 *    just held, press Y. Same with nectar, press A.
 *
 * 4. Write the four numbers into ShooterPivot: MIN_SAFE_POSITION,
 *    MAX_SAFE_POSITION, OPEN_POSITION, and the FALLBACK_*_POSITION pair. Or tell
 *    them to Claude and they will be put in with the provenance.
 *
 * WHICH DIRECTION IS "OPEN" IS NOT ASSUMED
 * Two files on this repo disagree about it. ShooterPivot's bench-measured
 * fallbacks put nectar at 0.135 and pollen at 0.48, meaning the LOWER number is
 * the wider gap -- nectar is the bigger ball. Jade's PieceDetector does the
 * opposite, sending nectar to 1.0 and pollen to 0.0. Those cannot both be right,
 * and PieceDetector's values look like untested full-range placeholders. This op
 * mode takes no side: you jog, you look, you label what you see.
 */
@Configurable
@TeleOp(name = "Pivot Jog Test", group = "Test")
public class PivotJogTest extends LinearOpMode {

    /** One press of a bumper. Small enough that a mistake cannot slam anything. */
    public static double FINE_STEP = 0.002;

    /** Held-trigger jog, in position units per second at full trigger. */
    public static double COARSE_RATE = 0.08;

    private double position;
    private boolean everCommanded = false;

    private double openEnd = Double.NaN;
    private double closedEnd = Double.NaN;
    private double pollenAt = Double.NaN;
    private double nectarAt = Double.NaN;

    private String note = "nothing commanded yet - jog one fine step to begin";

    private boolean lbPrev, rbPrev, xPrev, bPrev, yPrev, aPrev;
    private boolean upPrev, downPrev, leftPrev, rightPrev;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Servo pivot = hardwareMap.tryGet(Servo.class, "shooterPivot");

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

        // Whatever the servo last had. NOT commanded here -- see step 1.
        position = Range.clip(pivot.getPosition(), 0.0, 1.0);

        telemetry.addLine("PIVOT JOG TEST");
        telemetry.addLine("Beam break not used. Nothing moves until you jog.");
        telemetry.addLine();
        telemetry.addData("Servo reports", position);
        telemetry.addLine();
        telemetry.addLine("Bumpers: fine jog   Triggers: coarse jog (held)");
        telemetry.addLine("Record: X=open end  B=closed end  Y=pollen  A=nectar");
        telemetry.addLine("FIND THE ENDS FIRST. Back off as soon as it strains.");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            handleJog(dt);
            handleRecord();
            handleGoTo();

            if (everCommanded) {
                pivot.setPosition(position);
            }

            telemetry.addLine("Bumpers: fine   Triggers: coarse (held)");
            telemetry.addLine("Record: X=open  B=closed  Y=pollen  A=nectar");
            telemetry.addLine("Go to:  dpad U=open D=closed L=pollen R=nectar");
            telemetry.addLine();
            telemetry.addData("POSITION  <-- write this down", position);
            telemetry.addData("Commanded yet", everCommanded ? "yes" : "no - nothing has moved");
            telemetry.addLine();
            telemetry.addData("Open end   (X)", show(openEnd));
            telemetry.addData("Closed end (B)", show(closedEnd));
            telemetry.addData("Pollen     (Y)", show(pollenAt));
            telemetry.addData("Nectar     (A)", show(nectarAt));
            telemetry.addLine();
            telemetry.addData("Travel band", bandDescription());
            telemetry.addLine();
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
                    telemetry.addLine(String.format("  FALLBACK_POLLEN_POSITION = %.3f;", pollenAt));
                }
                if (!Double.isNaN(nectarAt)) {
                    telemetry.addLine(String.format("  FALLBACK_NECTAR_POSITION = %.3f;", nectarAt));
                }
            }
            telemetry.update();
        }
    }

    // ---- Input --------------------------------------------------------------

    private void handleJog(double dt) {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (rb && !rbPrev) {
            move(FINE_STEP);
        }
        if (lb && !lbPrev) {
            move(-FINE_STEP);
        }
        rbPrev = rb;
        lbPrev = lb;

        // Rate-based so a held trigger is steerable rather than a leap.
        double coarse = (gamepad1.right_trigger - gamepad1.left_trigger) * COARSE_RATE * dt;
        if (coarse != 0.0) {
            move(coarse);
        }
    }

    /** Applies a jog, clamped to whichever ends are known. */
    private void move(double delta) {
        double wanted = position + delta;
        double clamped = clampToKnownBand(wanted);
        if (clamped != wanted) {
            note = "held at a recorded end - will not jog past it";
        } else {
            note = "";
        }
        position = clamped;
        everCommanded = true;
    }

    private void handleRecord() {
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;

        if (x && !xPrev) {
            openEnd = position;
            note = String.format("OPEN end recorded at %.3f", position);
        }
        if (b && !bPrev) {
            closedEnd = position;
            note = String.format("CLOSED end recorded at %.3f", position);
        }
        if (y && !yPrev) {
            pollenAt = position;
            note = String.format("POLLEN recorded at %.3f", position);
        }
        if (a && !aPrev) {
            nectarAt = position;
            note = String.format("NECTAR recorded at %.3f", position);
        }

        xPrev = x;
        bPrev = b;
        yPrev = y;
        aPrev = a;
    }

    private void handleGoTo() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !upPrev) {
            goTo(openEnd, "open end");
        }
        if (down && !downPrev) {
            goTo(closedEnd, "closed end");
        }
        if (left && !leftPrev) {
            goTo(pollenAt, "pollen");
        }
        if (right && !rightPrev) {
            goTo(nectarAt, "nectar");
        }

        upPrev = up;
        downPrev = down;
        leftPrev = left;
        rightPrev = right;
    }

    private void goTo(double target, String name) {
        if (Double.isNaN(target)) {
            note = name + " has not been recorded yet";
            return;
        }
        position = clampToKnownBand(target);
        everCommanded = true;
        note = "moved to " + name;
    }

    // ---- Travel band --------------------------------------------------------

    /**
     * Clamps to the ends that have been recorded. One end recorded clamps on
     * that side only, so the ends can be found one at a time without the op
     * mode fighting the search.
     */
    private double clampToKnownBand(double wanted) {
        double result = Range.clip(wanted, 0.0, 1.0);
        if (bothEndsKnown()) {
            return Range.clip(result, lowEnd(), highEnd());
        }
        if (!Double.isNaN(openEnd)) {
            result = clampAgainstSingleEnd(result, openEnd);
        }
        if (!Double.isNaN(closedEnd)) {
            result = clampAgainstSingleEnd(result, closedEnd);
        }
        return result;
    }

    /**
     * With only one end known, which side is out of bounds is ambiguous -- so
     * this only blocks going FURTHER past it than the pivot already is.
     */
    private double clampAgainstSingleEnd(double wanted, double end) {
        if (position <= end && wanted > end) {
            return end;
        }
        if (position >= end && wanted < end) {
            return end;
        }
        return wanted;
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
            return String.format("%.3f to %.3f (jogging is limited to this)",
                    lowEnd(), highEnd());
        }
        if (Double.isNaN(openEnd) && Double.isNaN(closedEnd)) {
            return "no ends recorded - FULL range allowed, jog gently";
        }
        return "one end recorded - find the other before trusting the band";
    }

    private String show(double value) {
        return Double.isNaN(value) ? "not recorded" : String.format("%.3f", value);
    }
}
