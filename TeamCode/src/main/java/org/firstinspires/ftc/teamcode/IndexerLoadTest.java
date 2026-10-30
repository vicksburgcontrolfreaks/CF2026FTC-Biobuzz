package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Measures whether pollen and nectar can be told apart by how hard the indexer
 * has to work to push them through. A bigger ball is compressed more, so it
 * pushes back harder, so the motor draws more current -- that is the whole idea.
 *
 * THIS IS A MEASUREMENT TOOL, NOT A CLASSIFIER. It does not decide what a ball
 * is. It collects current samples for balls you label by hand, then tells you
 * whether the two groups actually separate. Build the classifier only if they
 * do -- no amount of code fixes overlapping distributions.
 *
 * WHY CURRENT AND NOT FORCE
 * There is no force sensor on the robot. For a DC motor torque is roughly
 * proportional to current, so current draw is the cheapest proxy for "how hard
 * is this?" available. The technique is called current sensing, or sensorless
 * load detection, because the motor is its own sensor.
 *
 * THE THREE THINGS THAT CORRUPT A CURRENT READING, AND WHAT THIS DOES ABOUT THEM
 *
 * 1. BATTERY VOLTAGE SAGS. The same torque draws different current at 13.5V
 *    than at 12.0V, and a pack droops across a match. Every sample is recorded
 *    with the voltage at that moment and normalized to 12V, so numbers taken on
 *    a fresh pack and a tired one are comparable.
 *
 * 2. BASELINE DRIFTS. Friction, dust and motor temperature all move the
 *    no-ball current. So what gets compared is the RISE ABOVE BASELINE, not the
 *    absolute amps. Press Y on an empty indexer to record baselines -- do a few,
 *    and redo them if the robot has been sitting.
 *
 * 3. ONE SAMPLE IS NOISE. Current spikes and dips as the wheel turns and the
 *    ball shifts. Each pass records the PEAK across the whole pass, which is far
 *    more repeatable than whatever the number happened to be at one instant.
 *
 * CONTROLS (gamepad1)
 *   A (hold)      run the indexer. Peak current is tracked for as long as you
 *                 hold it. One hold = one "pass".
 *   X             file the last pass as POLLEN
 *   B             file the last pass as NECTAR
 *   Y             file the last pass as BASELINE (run it empty first)
 *   LB (hold)     reverse the indexer, to clear a jam. Not recorded.
 *   D-pad up/down feed RPM +/- 10
 *   D-pad left    discard the last pass
 *   D-pad right   clear ALL samples and start over
 *
 * HOW TO RUN THE EXPERIMENT
 * 1. Empty indexer. Hold A for a second, release, press Y. Repeat 3 times.
 *    That is your baseline.
 * 2. Feed a pollen ball: hold A until it is through, release, press X.
 *    Do ten. Vary it a little -- different balls, different orientations.
 * 3. Same with nectar, pressing B. Do ten.
 * 4. Read the SEPARATION line at the bottom. That is the answer.
 *
 * WHAT THE ANSWER MEANS
 *   SEPARATED  the groups do not overlap. It gives you a threshold to classify
 *              on and the margin you have. Build it.
 *   OVERLAP    some pollen pushed as hard as some nectar. Current sensing
 *              cannot tell them apart reliably, and a classifier would just be
 *              guessing on the overlapping ones. Measure size instead.
 *
 * TIMING, FOR WHEN THIS WORKS
 * The ball travels about 6 inches up the ramp after the indexer, which at 50
 * RPM is roughly 0.6-1.2 seconds depending on wheel diameter -- more than the
 * 0.3-0.5s a servo needs to reposition the pivot. So sensing inline is fine at
 * this feed speed. If the indexer is ever sped up for match pace that margin
 * shrinks proportionally, and this stops being true around 200 RPM.
 *
 * The RPM math is done locally rather than through CollectionSystem, because
 * that class requires a color sensor this robot does not have fitted yet.
 */
@TeleOp(name = "Indexer Load Test", group = "Test")
public class IndexerLoadTest extends LinearOpMode {

    private static final double START_RPM = 50.0;
    private static final double STEP_RPM = 10.0;
    private static final double MAX_RPM = 300.0;

    /** Samples are scaled to this voltage so a tired pack reads like a fresh one. */
    private static final double NOMINAL_VOLTS = 12.0;

    /** Enough samples to mean something, few enough to bound memory. */
    private static final int MAX_SAMPLES = 60;

    /** Below this many samples per group, a separation verdict is not worth printing. */
    private static final int MIN_FOR_VERDICT = 3;

    private static final double TICKS_PER_MOTOR_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double INDEXER_RATIO = RobotHardware.INDEXER_GEAR_RATIO;

    private final List<Double> pollenPeaks = new ArrayList<>();
    private final List<Double> nectarPeaks = new ArrayList<>();
    private final List<Double> baselinePeaks = new ArrayList<>();

    private double targetRpm = START_RPM;

    // The pass currently being recorded.
    private boolean passActive = false;
    private double passPeakAmps = 0.0;
    private final ElapsedTime passTimer = new ElapsedTime();

    // The finished pass, waiting to be labeled.
    private boolean havePass = false;
    private double lastRawPeak = 0.0;
    private double lastNormPeak = 0.0;
    private double lastVolts = 0.0;
    private double lastDuration = 0.0;

    private String note = "";

    private boolean xPrev, bPrev, yPrev, upPrev, downPrev, leftPrev, rightPrev;

    @Override
    public void runOpMode() {
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        // Optional: the color sensor is not fitted yet, so tryGet keeps this op
        // mode runnable. If one does get fitted, its distance channel shows up on
        // telemetry for free and you can compare the two approaches side by side.
        NormalizedColorSensor colorSensor =
                hardwareMap.tryGet(NormalizedColorSensor.class, "colorSensor");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("INDEXER LOAD TEST");
        telemetry.addLine();
        telemetry.addLine("Hold A to feed. Release, then label it:");
        telemetry.addLine("  X = pollen    B = nectar    Y = baseline (empty)");
        telemetry.addLine();
        telemetry.addLine("Do 3 empty baselines FIRST, then 10 of each ball.");
        telemetry.addData("Color sensor", colorSensor != null ? "found" : "not fitted (fine)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double volts = battery.getVoltage();

            // ---- Run the indexer, tracking the peak across the hold ----------
            if (gamepad1.a) {
                if (!passActive) {
                    passActive = true;
                    passPeakAmps = 0.0;
                    passTimer.reset();
                }
                indexer.setVelocity(rpmToTicksPerSec(targetRpm));
                double amps = indexer.getCurrent(CurrentUnit.AMPS);
                passPeakAmps = Math.max(passPeakAmps, amps);
            } else if (gamepad1.left_bumper) {
                endPassIfActive(volts);
                indexer.setVelocity(rpmToTicksPerSec(-targetRpm));
            } else {
                endPassIfActive(volts);
                indexer.setPower(0.0);
            }

            handleLabeling();
            handleRpmAdjust();

            // ---- Telemetry ---------------------------------------------------
            telemetry.addLine("A: feed   X: pollen   B: nectar   Y: baseline");
            telemetry.addLine("Dpad U/D: RPM +/-10   L: discard last   R: clear all");
            if (!note.isEmpty()) {
                telemetry.addLine();
                telemetry.addData(">>", note);
            }
            telemetry.addLine();
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Actual RPM", ticksPerSecToRpm(indexer.getVelocity()));
            telemetry.addData("Amps now", indexer.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Peak this pass", passActive ? passPeakAmps : 0.0);
            telemetry.addData("Battery V", volts);

            telemetry.addLine();
            if (havePass) {
                telemetry.addLine("-- LAST PASS, waiting for a label --");
                telemetry.addData("  Peak raw amps", lastRawPeak);
                telemetry.addData("  Peak at 12V", lastNormPeak);
                telemetry.addData("  Held for (s)", lastDuration);
            } else {
                telemetry.addLine("-- no unlabeled pass --");
            }

            telemetry.addLine();
            double baseMean = mean(baselinePeaks);
            telemetry.addData("Baselines", summary(baselinePeaks, 0.0));
            telemetry.addData("Pollen", summary(pollenPeaks, baseMean));
            telemetry.addData("Nectar", summary(nectarPeaks, baseMean));

            telemetry.addLine();
            telemetry.addLine(verdict(baseMean));

            if (colorSensor != null) {
                telemetry.addLine();
                telemetry.addData("Color (raw argb)",
                        colorSensor.getNormalizedColors().toColor());
            }
            telemetry.update();
        }

        indexer.setPower(0.0);
    }

    /** Freezes the pass that just ended so it can be labeled. */
    private void endPassIfActive(double volts) {
        if (!passActive) {
            return;
        }
        passActive = false;
        // A tap too short to mean anything is thrown away rather than recorded.
        if (passTimer.seconds() < 0.15) {
            note = "pass too short to count - hold A until the ball is through";
            return;
        }
        lastRawPeak = passPeakAmps;
        lastVolts = volts;
        lastNormPeak = passPeakAmps * (NOMINAL_VOLTS / Math.max(volts, 1.0));
        lastDuration = passTimer.seconds();
        havePass = true;
        note = "pass recorded - now press X, B or Y to label it";
    }

    private void handleLabeling() {
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (x && !xPrev) {
            file(pollenPeaks, "POLLEN");
        }
        if (b && !bPrev) {
            file(nectarPeaks, "NECTAR");
        }
        if (y && !yPrev) {
            file(baselinePeaks, "BASELINE");
        }
        if (left && !leftPrev) {
            havePass = false;
            note = "last pass discarded";
        }
        if (right && !rightPrev) {
            pollenPeaks.clear();
            nectarPeaks.clear();
            baselinePeaks.clear();
            havePass = false;
            note = "ALL samples cleared";
        }

        xPrev = x;
        bPrev = b;
        yPrev = y;
        leftPrev = left;
        rightPrev = right;
    }

    private void file(List<Double> into, String name) {
        if (!havePass) {
            note = "nothing to label - hold A to run a pass first";
            return;
        }
        if (into.size() >= MAX_SAMPLES) {
            note = name + " is full at " + MAX_SAMPLES + " samples";
            return;
        }
        into.add(lastNormPeak);
        havePass = false;
        note = "filed as " + name + " (" + into.size() + " so far)";
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

    // ---- Stats --------------------------------------------------------------

    /**
     * The separation verdict. Compares the two groups' rise above baseline and
     * says plainly whether a threshold exists that splits them.
     */
    private String verdict(double baseMean) {
        if (pollenPeaks.size() < MIN_FOR_VERDICT || nectarPeaks.size() < MIN_FOR_VERDICT) {
            return "SEPARATION: need " + MIN_FOR_VERDICT + "+ of each ball"
                    + " (pollen " + pollenPeaks.size()
                    + ", nectar " + nectarPeaks.size() + ")";
        }
        if (baselinePeaks.isEmpty()) {
            return "SEPARATION: record a baseline first (empty indexer, A then Y)";
        }

        double pLo = min(pollenPeaks) - baseMean;
        double pHi = max(pollenPeaks) - baseMean;
        double nLo = min(nectarPeaks) - baseMean;
        double nHi = max(nectarPeaks) - baseMean;

        if (pHi < nLo) {
            double margin = nLo - pHi;
            return String.format("SEPARATED by %.2fA. Threshold %.2fA above baseline"
                    + " (below = pollen).", margin, (pHi + nLo) / 2.0);
        }
        if (nHi < pLo) {
            double margin = pLo - nHi;
            return String.format("SEPARATED by %.2fA. Threshold %.2fA above baseline"
                    + " (below = nectar).", margin, (nHi + pLo) / 2.0);
        }
        double overlap = Math.min(pHi, nHi) - Math.max(pLo, nLo);
        return String.format("OVERLAP of %.2fA - not separable. Measure size instead.",
                overlap);
    }

    /** "n=10  rise 1.20 / 1.45 / 1.80 A (min/mean/max)" */
    private String summary(List<Double> values, double baseMean) {
        if (values.isEmpty()) {
            return "none yet";
        }
        return String.format("n=%d  %.2f / %.2f / %.2f A",
                values.size(),
                min(values) - baseMean,
                mean(values) - baseMean,
                max(values) - baseMean);
    }

    private double mean(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double total = 0.0;
        for (double v : values) {
            total += v;
        }
        return total / values.size();
    }

    private double min(List<Double> values) {
        double lowest = Double.MAX_VALUE;
        for (double v : values) {
            lowest = Math.min(lowest, v);
        }
        return lowest;
    }

    private double max(List<Double> values) {
        double highest = -Double.MAX_VALUE;
        for (double v : values) {
            highest = Math.max(highest, v);
        }
        return highest;
    }

    // ---- Units --------------------------------------------------------------

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_MOTOR_REV * INDEXER_RATIO / 60.0;
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / (TICKS_PER_MOTOR_REV * INDEXER_RATIO)) * 60.0;
    }
}
