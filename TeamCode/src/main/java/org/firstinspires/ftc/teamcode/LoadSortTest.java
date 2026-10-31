package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.BallLoadClassifier;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * The whole chain, end to end: feed a ball, classify it from the indexer's
 * current, and set the shooter pivot to match. Triggered -- you say when a ball
 * is passing by holding A.
 *
 * Built on the calibration measured on 2026-10-30; the numbers and what
 * invalidates them live in BallLoadClassifier, not here.
 *
 * WHAT IT NEEDS IN THE CONFIG
 *   "indexer"       DcMotorEx, required
 *   "shooterPivot"  Servo, OPTIONAL -- all the pivot half needs
 *   "pivotHome"     DigitalChannel, OPTIONAL and unused here; nothing homes
 * Without the pivot servo it still classifies and reports what it WOULD
 * command, which is how you check the logic before anything moves.
 *
 * THE PIVOT OPENS FIRST, THEN COMMITS
 * A nectar ball does not fit a gap set for pollen, so the pivot cannot be left
 * at the last ball's spacing while an unknown one is on its way. Two things stop
 * that. The pivot widens the moment a pass STARTS, before anything is known --
 * too wide only fails to grip, while too narrow jams. And nectar is acted on the
 * instant the current crosses the threshold, not when the trigger is released.
 * Both are safe to do mid-pass because the indexer sits upstream of the pivot
 * with ~6 inches of ramp between them, so the pivot is nowhere near the ball
 * being measured.
 *
 * CONTROLS (gamepad1)
 *   A (hold)      feed a ball. The pivot opens on press; nectar is caught
 *                 mid-pass; pollen is set when you RELEASE.
 *   LB (hold)     reverse the indexer to clear a jam. Not classified.
 *   Y             open the pivot wide, to pull a stuck ball out. USE THIS
 *                 rather than moving the pivot by hand -- back-driving a
 *                 powered servo is what wrecked one on 2026-10-30.
 *   X / B         force the pivot to pollen / nectar by hand, to compare
 *   D-pad up/down feed RPM +/- 10
 *
 * HOW TO CHECK IT
 *
 * 1. Watch "Would command" with no pivot fitted, or "Pivot position" with one.
 *    Feed a pollen ball and a nectar ball and confirm the verdict matches what
 *    you actually fed. Ten of each is enough to believe it.
 *
 * 2. Watch "Margin from threshold". Every pass in the calibration run sat at
 *    least 0.66 A clear of the threshold. If you start seeing margins under
 *    about 0.2 A, the calibration has drifted -- re-run Indexer Load Test
 *    rather than nudging the threshold on a hunch.
 *
 * 3. THE PIVOT IS USING MEASURED ABSOLUTE POSITIONS, and that is the design,
 *    not a degraded mode. A servo's setPosition is absolute, so no homing is
 *    needed: goToPollen/goToNectar go to the positions measured against real
 *    balls on 2026-10-30 -- pollen 0.539, nectar 0.21, inside a safe band of
 *    0.166 to 0.679. "Pivot positions from" says which numbers are in play.
 *    Re-measure with Pivot Jog Test after any work on the pivot linkage; the
 *    reasoning is in ShooterPivot's class comment.
 *
 * WHAT THIS CANNOT DO
 * POLLEN_OR_EMPTY means exactly that. A pollen ball and an empty indexer both
 * read baseline, so an empty feed reports POLLEN_OR_EMPTY and parks the pivot at
 * pollen spacing. Harmless here, but do not build ball counting on this.
 */
@Configurable
@TeleOp(name = "Load Sort Test", group = "Test")
public class LoadSortTest extends LinearOpMode {

    public static double START_RPM = 50.0;
    public static double STEP_RPM = 10.0;
    public static double MAX_RPM = 300.0;

    private static final double TICKS_PER_MOTOR_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double INDEXER_RATIO = RobotHardware.INDEXER_GEAR_RATIO;

    private double targetRpm = START_RPM;

    private int pollenCount, nectarCount, noPassCount;
    private String lastAction = "nothing fed yet";

    /**
     * Set once per pass, the moment the live rise crosses the nectar threshold.
     * One-way on purpose: the classifier tracks a PEAK, which only ever rises,
     * so a crossing can never un-cross and this cannot chatter.
     */
    private boolean nectarLatched;

    private boolean xPrev, bPrev, yPrev, upPrev, downPrev;

    @Override
    public void runOpMode() {
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        BallLoadClassifier classifier = new BallLoadClassifier(indexer, battery);

        // Optional pivot: the servo is all it takes. The pivotHome beam break is
        // passed along if the config happens to have one, but ShooterPivot
        // tolerates null and never homes here either way.
        Servo pivotServo = hardwareMap.tryGet(Servo.class, "shooterPivot");
        DigitalChannel pivotBeam = hardwareMap.tryGet(DigitalChannel.class, "pivotHome");
        ShooterPivot pivot = pivotServo != null
                ? new ShooterPivot(pivotServo, pivotBeam)
                : null;

        telemetry.addLine("LOAD SORT TEST");
        telemetry.addLine("Hold A to feed a ball. Verdict lands when you release.");
        telemetry.addLine();
        telemetry.addData("Pivot", pivot != null
                ? "fitted - it will actually move"
                : "not fitted - will only report what it would command");
        telemetry.addData("Threshold", BallLoadClassifier.NECTAR_RISE_AMPS
                + "A above a " + BallLoadClassifier.BASELINE_AMPS + "A baseline");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- Feed, classify on release -------------------------------
            if (gamepad1.a) {
                if (!classifier.isPassActive()) {
                    classifier.startPass();
                    openForUnknownBall(pivot);
                }
                indexer.setVelocity(rpmToTicksPerSec(targetRpm));
                classifier.update();
                catchNectarMidPass(classifier, pivot);
            } else {
                boolean wasFeeding = classifier.isPassActive();
                indexer.setPower(0.0);

                if (gamepad1.left_bumper) {
                    if (wasFeeding) {
                        classifier.cancelPass();
                        nectarLatched = false;
                    }
                    indexer.setVelocity(rpmToTicksPerSec(-targetRpm));
                } else if (wasFeeding) {
                    applyVerdict(classifier.endPass(), pivot);
                }
            }

            handleManualPivot(pivot);
            handleRpmAdjust();

            // ---- Telemetry ------------------------------------------------
            telemetry.addLine("A: feed   LB: reverse   Y: OPEN   X/B: force pollen/nectar");
            telemetry.addLine("Dpad U/D: feed RPM +/-10");
            telemetry.addLine();
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Amps now", indexer.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine();
            if (classifier.isPassActive()) {
                telemetry.addLine("-- PASS IN PROGRESS --");
                telemetry.addData("  Peak so far (12V)", classifier.getPeakNormAmps());
                telemetry.addData("  Rise above baseline", classifier.getRiseAmps());
                telemetry.addData("  Nectar caught early", nectarLatched
                        ? "YES - pivot already at nectar"
                        : "not yet - needs " + BallLoadClassifier.NECTAR_RISE_AMPS + "A");
            } else {
                telemetry.addLine("-- LAST VERDICT --");
                telemetry.addData("  Result", classifier.getLastResult());
                telemetry.addData("  Peak (12V)", classifier.getPeakNormAmps());
                telemetry.addData("  Rise above baseline", classifier.getLastRiseAmps());
                telemetry.addData("  Margin from threshold", classifier.getLastMarginAmps());
                telemetry.addData("  Pass length (s)", classifier.getLastPassSeconds());
            }

            telemetry.addLine();
            telemetry.addData("Action", lastAction);
            telemetry.addData("Tally", "pollen/empty " + pollenCount
                    + "   nectar " + nectarCount
                    + "   too short " + noPassCount);

            telemetry.addLine();
            if (pivot != null) {
                telemetry.addData("Pivot position", pivot.getCommandedPosition());
                telemetry.addData("Pivot positions from",
                        pivot.isUsingHomedOffsets()
                                ? "homed zero + offsets"
                                : "measured absolutes (pollen "
                                        + ShooterPivot.POLLEN_POSITION + ", nectar "
                                        + ShooterPivot.NECTAR_POSITION + ")");
            } else {
                telemetry.addLine("Pivot not fitted - nothing moved");
            }
            telemetry.update();
        }

        indexer.setPower(0.0);
    }

    /**
     * Widens the pivot before the ball gets anywhere near it, at the moment the
     * pass starts and nothing is known about what is coming.
     *
     * A nectar ball cannot fit a gap set for pollen. Waiting for the verdict to
     * set the gap means a nectar arriving while the pivot still sits at the last
     * ball's pollen spacing (0.539) finds it 0.329 of travel too tight, and
     * jams. Opening first makes the gap wrong in the SAFE direction -- too wide
     * never jams, it just does not grip, and the verdict closes it in time.
     *
     * It also puts the timing margin where it is needed. OPEN_POSITION (0.17) is
     * only 0.04 from the nectar grip (0.21), so nectar needs almost no servo
     * travel after it is recognized, and the whole ~0.6-1.2s of ramp travel is
     * available for the longer pollen move (0.17 -> 0.539) -- the case that can
     * afford to be late, since a pollen ball fits any gap.
     */
    private void openForUnknownBall(ShooterPivot pivot) {
        nectarLatched = false;
        if (pivot == null) {
            lastAction = "pass started -> would open the pivot for an unknown ball";
        } else if (pivot.goToOpen()) {
            lastAction = "pass started -> pivot opened for an unknown ball";
        } else {
            lastAction = "pass started -> OPEN_POSITION not set, pivot left alone";
        }
    }

    /**
     * Commits to nectar spacing the instant the live rise crosses the threshold,
     * instead of waiting for A to be released.
     *
     * Safe to act on mid-pass because the indexer is UPSTREAM of the pivot, with
     * about 6 inches of ramp in between, so moving the pivot does not disturb
     * the current being measured. And the latch cannot chatter: the classifier
     * tracks a peak, so once the threshold is crossed it stays crossed.
     *
     * With openForUnknownBall() above this is only about 0.01 of travel, so its
     * real job is not the move -- it is not having to wait for the trigger
     * release, which may come long after the ball is actually through.
     */
    private void catchNectarMidPass(BallLoadClassifier classifier, ShooterPivot pivot) {
        if (nectarLatched || classifier.getRiseAmps() < BallLoadClassifier.NECTAR_RISE_AMPS) {
            return;
        }
        nectarLatched = true;
        if (pivot != null) {
            pivot.goToNectar();
            lastAction = "NECTAR seen mid-pass -> pivot to nectar spacing NOW";
        } else {
            lastAction = "NECTAR seen mid-pass -> would command nectar spacing NOW";
        }
    }

    /** Sets the pivot from the verdict, or says what it would have set. */
    private void applyVerdict(BallLoadClassifier.Result result, ShooterPivot pivot) {
        switch (result) {
            case NECTAR:
                nectarCount++;
                if (nectarLatched) {
                    // catchNectarMidPass() already moved it. Re-commanding would
                    // be harmless, but saying so is how you confirm on the robot
                    // that the early path is the one doing the work.
                    lastAction = "NECTAR -> already set mid-pass, nothing to do";
                } else if (pivot != null) {
                    pivot.goToNectar();
                    lastAction = "NECTAR -> pivot to nectar spacing (on release)";
                } else {
                    lastAction = "NECTAR -> would command nectar spacing (on release)";
                }
                break;

            case POLLEN_OR_EMPTY:
                pollenCount++;
                if (pivot != null) {
                    pivot.goToPollen();
                    lastAction = "POLLEN (or empty) -> pivot to pollen spacing";
                } else {
                    lastAction = "POLLEN (or empty) -> would command pollen spacing";
                }
                break;

            case NO_PASS:
            default:
                noPassCount++;
                // The pivot is NOT where it started: openForUnknownBall() widened
                // it when the pass began. Leaving it open is the right call for a
                // pass nobody can judge -- open does not grip, but it cannot jam.
                lastAction = nectarLatched
                        ? "pass too short to judge, but nectar load was seen - left at nectar"
                        : "pass too short to judge - pivot left open";
                break;
        }
    }

    private void handleManualPivot(ShooterPivot pivot) {
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;
        if (x && !xPrev && pivot != null) {
            pivot.goToPollen();
            lastAction = "forced to pollen spacing by hand";
        }
        if (b && !bPrev && pivot != null) {
            pivot.goToNectar();
            lastAction = "forced to nectar spacing by hand";
        }
        if (y && !yPrev && pivot != null) {
            lastAction = pivot.goToOpen()
                    ? "OPEN - safe to pull a ball out"
                    : "open position not configured";
        }
        xPrev = x;
        bPrev = b;
        yPrev = y;
    }

    private void handleRpmAdjust() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !upPrev) {
            targetRpm += STEP_RPM;
        }
        if (down && !downPrev) {
            targetRpm -= STEP_RPM;
        }
        targetRpm = Range.clip(targetRpm, 0.0, MAX_RPM);
        upPrev = up;
        downPrev = down;
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_MOTOR_REV * INDEXER_RATIO / 60.0;
    }
}
