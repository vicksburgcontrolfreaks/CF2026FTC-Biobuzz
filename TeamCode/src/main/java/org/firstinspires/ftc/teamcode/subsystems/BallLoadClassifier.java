package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Tells nectar from pollen by how hard the indexer has to work to push the ball
 * through. A nectar ball is bigger, so it is compressed more, so it pushes back
 * harder, so the motor draws more current. The technique is current sensing, or
 * sensorless load detection -- the motor is its own sensor, because there is no
 * force sensor on the robot.
 *
 * MEASURED ON THE ROBOT 2026-10-30, at 50 RPM indexer, normalized to 12V.
 * Twenty labeled passes with Indexer Load Test, ten of each ball:
 *
 *   empty indexer   n=3   peak 0.71 / 0.79 / 0.86 A   (min / mean / max)
 *   pollen          n=10  rise -0.24 / -0.06 / 0.06 A above baseline
 *   nectar          n=10  rise  1.38 /  2.09 / 2.63 A above baseline
 *
 *   gap between the groups: 1.32 A
 *
 * The gap is WIDER than either group's own spread (0.30 A for pollen, 1.25 A
 * for nectar), which is the test that actually matters -- the separation is
 * bigger than the noise it has to survive. The threshold sits at the midpoint,
 * so either kind of misclassification needs a 0.66 A excursion to happen.
 *
 * WHY THE RESULT IS CALLED POLLEN_OR_EMPTY
 * Pollen's rise above baseline measured -0.06 A on average. That is zero within
 * noise: a pollen ball costs the indexer essentially nothing. Which is good news
 * for reliability -- this is really answering "is there extra load or not", not
 * discriminating two load levels, and that is a much easier question.
 *
 * But it means current sensing CANNOT tell pollen from no ball at all. Both read
 * baseline. The result is named to make that impossible to forget, because the
 * magazine beam breaks were retired on 2026-10-27 and nothing replaced them, so
 * there is no ball-presence signal anywhere on the robot right now. Anything
 * that needs to know a ball EXISTS -- counting, deciding whether to spin up,
 * knowing the magazine is loaded -- needs a different sensor, not this class.
 *
 * THREE THINGS THAT INVALIDATE THE CALIBRATION
 *
 * 1. FEED SPEED. Every number above is at 50 RPM. Current scales with load and
 *    speed together, so changing the feed rate moves the threshold with it.
 *    Re-run Indexer Load Test if the indexer speed changes.
 *
 * 2. HOW THE PASS STARTS. On a light pass the peak is probably the motor's
 *    startup inrush rather than the ball -- that is why pollen measured slightly
 *    BELOW an empty run, which a ball cannot actually cause. The threshold is
 *    therefore calibrated against starting from rest. If feeding ever changes so
 *    the indexer is already at speed when a ball arrives, the baseline shifts.
 *
 * 3. MECHANICAL CHANGES. New wheels, a different ramp angle, scuffed balls, or a
 *    warm versus cold mechanism all move the numbers. The 1.32 A gap buys a lot
 *    of room, but it is not infinite.
 *
 * HOW TO USE IT (triggered -- the caller says when a ball is passing)
 *
 *     classifier.startPass();
 *     while (feeding) {
 *         indexer.setVelocity(...);
 *         classifier.update();        // every loop
 *     }
 *     Result r = classifier.endPass();
 *     if (r == Result.NECTAR) pivot.goToNectar(); else pivot.goToPollen();
 *
 * There is time for that: about 6 inches of ramp travel after the indexer, which
 * at 50 RPM is roughly 0.6-1.2 seconds, against the 0.3-0.5 seconds a servo
 * needs to reposition. That margin shrinks as feed speed rises and runs out
 * somewhere near 200 RPM.
 */
@Configurable
public class BallLoadClassifier {

    public enum Result {
        /** Load well above baseline. Nectar, confidently. */
        NECTAR,
        /**
         * Load at or near baseline. A pollen ball -- OR nothing at all. Current
         * sensing cannot separate those two; see the class notes.
         */
        POLLEN_OR_EMPTY,
        /** The pass was too short to judge. Nothing was classified. */
        NO_PASS
    }

    /** Empty-indexer peak, normalized to 12V. Measured mean of 3 on 2026-10-30. */
    public static double BASELINE_AMPS = 0.79;

    /**
     * Rise above baseline that counts as nectar. Midpoint of the measured gap
     * (pollen topped out at 0.06 A, nectar bottomed at 1.38 A).
     */
    public static double NECTAR_RISE_AMPS = 0.72;

    /** Samples are scaled to this voltage so a tired pack reads like a fresh one. */
    public static double NOMINAL_VOLTS = 12.0;

    /** Shorter than this and the pass is treated as a stray tap, not data. */
    public static double MIN_PASS_SECONDS = 0.15;

    private final DcMotorEx indexer;
    private final VoltageSensor battery;

    private final ElapsedTime passTimer = new ElapsedTime();
    private boolean passActive = false;
    private double peakNormAmps = 0.0;

    // Frozen at endPass() so telemetry can keep showing the last verdict.
    private Result lastResult = Result.NO_PASS;
    private double lastPeakNormAmps = 0.0;
    private double lastPassSeconds = 0.0;

    public BallLoadClassifier(DcMotorEx indexer, VoltageSensor battery) {
        this.indexer = indexer;
        this.battery = battery;
    }

    /** Call when the ball starts moving through the indexer. */
    public void startPass() {
        passActive = true;
        peakNormAmps = 0.0;
        passTimer.reset();
    }

    /**
     * Call every loop while the pass is running. Tracks the PEAK rather than an
     * instantaneous reading, because current jitters as the wheel turns and the
     * ball shifts -- the peak across a pass is far more repeatable.
     *
     * Each sample is voltage-normalized as it is taken, so sag during the pass
     * itself is corrected too. That is slightly different from how Indexer Load
     * Test calibrated the threshold (it normalized once, at pass end), but the
     * difference is 2-3% at most -- well inside the 1.32 A margin.
     */
    public void update() {
        if (!passActive) {
            return;
        }
        double volts = Math.max(battery.getVoltage(), 1.0);
        double normAmps = indexer.getCurrent(CurrentUnit.AMPS) * (NOMINAL_VOLTS / volts);
        peakNormAmps = Math.max(peakNormAmps, normAmps);
    }

    /** Call when the ball is through. Returns what it was. */
    public Result endPass() {
        if (!passActive) {
            return Result.NO_PASS;
        }
        passActive = false;
        lastPassSeconds = passTimer.seconds();
        lastPeakNormAmps = peakNormAmps;

        if (lastPassSeconds < MIN_PASS_SECONDS) {
            lastResult = Result.NO_PASS;
        } else if (getLastRiseAmps() >= NECTAR_RISE_AMPS) {
            lastResult = Result.NECTAR;
        } else {
            lastResult = Result.POLLEN_OR_EMPTY;
        }
        return lastResult;
    }

    /** Abandons the pass in progress without classifying it. */
    public void cancelPass() {
        passActive = false;
    }

    // ---- Readouts -----------------------------------------------------------

    public boolean isPassActive() {
        return passActive;
    }

    /** Live peak for the pass in progress, normalized to 12V. */
    public double getPeakNormAmps() {
        return passActive ? peakNormAmps : lastPeakNormAmps;
    }

    /** Live rise above baseline for the pass in progress. */
    public double getRiseAmps() {
        return getPeakNormAmps() - BASELINE_AMPS;
    }

    public double getLastRiseAmps() {
        return lastPeakNormAmps - BASELINE_AMPS;
    }

    public Result getLastResult() {
        return lastResult;
    }

    public double getLastPassSeconds() {
        return lastPassSeconds;
    }

    /**
     * How far the last pass sat from the threshold. Small numbers mean the call
     * was close, and a run of close calls means the calibration has drifted --
     * every measured pass was at least 0.66 A clear of it.
     */
    public double getLastMarginAmps() {
        return Math.abs(getLastRiseAmps() - NECTAR_RISE_AMPS);
    }
}
