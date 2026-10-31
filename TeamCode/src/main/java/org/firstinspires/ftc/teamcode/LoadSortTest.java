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
 * CONTROLS (gamepad1)
 *   A (hold)      feed a ball. Classification happens when you RELEASE.
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
                }
                indexer.setVelocity(rpmToTicksPerSec(targetRpm));
                classifier.update();
            } else {
                boolean wasFeeding = classifier.isPassActive();
                indexer.setPower(0.0);

                if (gamepad1.left_bumper) {
                    if (wasFeeding) {
                        classifier.cancelPass();
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

    /** Sets the pivot from the verdict, or says what it would have set. */
    private void applyVerdict(BallLoadClassifier.Result result, ShooterPivot pivot) {
        switch (result) {
            case NECTAR:
                nectarCount++;
                if (pivot != null) {
                    pivot.goToNectar();
                    lastAction = "NECTAR -> pivot to nectar spacing";
                } else {
                    lastAction = "NECTAR -> would command nectar spacing";
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
                lastAction = "pass too short to judge - pivot left alone";
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
