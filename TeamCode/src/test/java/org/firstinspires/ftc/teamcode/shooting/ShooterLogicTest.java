package org.firstinspires.ftc.teamcode.shooting;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * Tests for ShooterLogic, with a fake clock and made-up wheel speeds.
 * Run with:  gradlew :TeamCode:testDebugUnitTest
 *
 * Tuning constants are pinned in setUp() so editing the real defaults never
 * breaks these, and restored afterward.
 */
public class ShooterLogicTest {

    private static final double DT = 0.02;   // a 50 Hz loop
    private static final double EPS = 1e-9;

    private ShooterLogic shooter;
    private double now;

    private double savedTol, savedSettle, savedFeed, savedCont, savedMinRec, savedWarn, savedDisagree;
    private ShooterLogic.FeedMode savedMode;

    @Before public void setUp() {
        savedTol = ShooterLogic.TOLERANCE_RPM;
        savedSettle = ShooterLogic.SETTLE_SEC;
        savedFeed = ShooterLogic.FEED_SEC;
        savedCont = ShooterLogic.CONTINUOUS_SEC_PER_PIECE;
        savedMinRec = ShooterLogic.MIN_RECOVERY_SEC;
        savedWarn = ShooterLogic.SPINUP_WARN_SEC;
        savedDisagree = ShooterLogic.DISAGREE_RPM;
        savedMode = ShooterLogic.FEED_MODE;

        ShooterLogic.TOLERANCE_RPM = 50;
        ShooterLogic.SETTLE_SEC = 0.10;
        ShooterLogic.FEED_SEC = 0.20;
        ShooterLogic.CONTINUOUS_SEC_PER_PIECE = 0.30;
        ShooterLogic.MIN_RECOVERY_SEC = 0.10;
        ShooterLogic.SPINUP_WARN_SEC = 1.0;
        ShooterLogic.DISAGREE_RPM = 100;
        ShooterLogic.FEED_MODE = ShooterLogic.FeedMode.PULSE;

        shooter = new ShooterLogic();
        now = 0;
    }

    @After public void tearDown() {
        ShooterLogic.TOLERANCE_RPM = savedTol;
        ShooterLogic.SETTLE_SEC = savedSettle;
        ShooterLogic.FEED_SEC = savedFeed;
        ShooterLogic.CONTINUOUS_SEC_PER_PIECE = savedCont;
        ShooterLogic.MIN_RECOVERY_SEC = savedMinRec;
        ShooterLogic.SPINUP_WARN_SEC = savedWarn;
        ShooterLogic.DISAGREE_RPM = savedDisagree;
        ShooterLogic.FEED_MODE = savedMode;
    }

    /** Run the loop for `sec` seconds with both wheels at `rpm`. */
    private void run(double sec, double rpm) {
        run(sec, rpm, rpm);
    }

    private void run(double sec, double left, double right) {
        int steps = (int) Math.round(sec / DT);
        for (int i = 0; i < steps; i++) {
            now += DT;
            shooter.update(now, left, right);
        }
    }

    /** Wheels at speed long enough to be READY. */
    private void getReady(double rpm) {
        shooter.spinUp(rpm);
        run(0.5, rpm);
        assertEquals(ShooterLogic.State.READY, shooter.getState());
    }

    // ---------------- spin-up ----------------

    @Test public void startsIdleWithEverythingOff() {
        run(0.1, 0);
        assertEquals(ShooterLogic.State.IDLE, shooter.getState());
        assertEquals(0, shooter.getTargetRpm(), EPS);
        assertFalse(shooter.ownsIndexer());
        assertFalse(shooter.isBusy());
    }

    @Test public void readyOnlyAfterSettling() {
        shooter.spinUp(2000);
        assertEquals(2000, shooter.getTargetRpm(), EPS);
        run(0.5, 1500);                                  // nowhere near
        assertEquals(ShooterLogic.State.SPINNING_UP, shooter.getState());
        run(0.06, 2000);                                 // at speed, but not for SETTLE_SEC yet
        assertEquals(ShooterLogic.State.SPINNING_UP, shooter.getState());
        run(0.10, 2000);
        assertEquals(ShooterLogic.State.READY, shooter.getState());
        assertTrue(shooter.isReady());
    }

    @Test public void overshootPassingThroughToleranceIsNotReady() {
        shooter.spinUp(2000);
        run(0.04, 2000);   // passes through the band for 2 loops...
        run(0.20, 2200);   // ...and keeps going
        assertEquals(ShooterLogic.State.SPINNING_UP, shooter.getState());
    }

    @Test public void bothWheelsMustBeAtSpeed() {
        shooter.spinUp(2000);
        run(0.5, 2000, 1800);
        assertEquals(ShooterLogic.State.SPINNING_UP, shooter.getState());
        assertTrue(shooter.doWheelsDisagree());
    }

    @Test public void slowSpinUpIsFlaggedButKeepsTrying() {
        shooter.spinUp(2000);
        run(1.2, 1700);
        assertTrue(shooter.isSpinUpSlow());
        assertEquals(2000, shooter.getTargetRpm(), EPS);
        run(0.3, 2000);
        assertEquals(ShooterLogic.State.READY, shooter.getState());
        assertFalse(shooter.isSpinUpSlow());
    }

    @Test public void changingTheTargetDropsOutOfReady() {
        getReady(2000);
        shooter.spinUp(2400);           // drove farther away
        run(0.04, 2000);
        assertEquals(ShooterLogic.State.SPINNING_UP, shooter.getState());
        run(0.3, 2400);
        assertEquals(ShooterLogic.State.READY, shooter.getState());
    }

    // ---------------- firing ----------------

    @Test public void fireWhileIdleIsIgnored() {
        shooter.fire(3);
        run(1.0, 0);
        assertEquals(0, shooter.getShotsQueued());
        assertFalse(shooter.ownsIndexer());
    }

    @Test public void fireBeforeReadyWaitsForSpeed() {
        shooter.spinUp(2000);
        shooter.fire(1);
        assertTrue(shooter.isBusy());
        run(0.5, 1500);
        assertFalse("must not feed a slow wheel", shooter.ownsIndexer());
        run(0.2, 2000);
        assertTrue(shooter.ownsIndexer());
        assertEquals(1.0, shooter.getIndexerPower(), EPS);
    }

    @Test public void oneShotFeedsForFeedSecThenLetsGoOfTheIndexer() {
        getReady(2000);
        shooter.fire(1);
        run(0.02, 2000);
        assertTrue(shooter.ownsIndexer());
        run(0.20, 1850);                 // the shot sags the wheels
        assertFalse(shooter.ownsIndexer());
        assertEquals(ShooterLogic.State.RECOVERING, shooter.getState());
        assertEquals(1, shooter.getShotsFired());
        run(0.3, 2000);
        assertEquals(ShooterLogic.State.READY, shooter.getState());
        assertFalse(shooter.isBusy());
        assertEquals(150, shooter.getLastDipRpm(), EPS);
    }

    @Test public void threeShotsWaitForRecoveryBetweenEach() {
        getReady(2000);
        shooter.fire(3);
        int feeds = 0;
        boolean wasFeeding = false;
        // The wheels sag while feeding and stay low for 0.3 s after each shot.
        for (int i = 0; i < 300 && shooter.isBusy(); i++) {
            now += DT;
            boolean feeding = shooter.ownsIndexer();
            if (feeding && !wasFeeding) feeds++;
            wasFeeding = feeding;
            double rpm = shooter.getState() == ShooterLogic.State.FEEDING
                    || (shooter.getState() == ShooterLogic.State.RECOVERING && i % 25 < 15)
                    ? 1800 : 2000;
            shooter.update(now, rpm, rpm);
        }
        assertEquals(3, feeds);
        assertEquals(3, shooter.getShotsFired());
        assertFalse(shooter.isBusy());
    }

    @Test public void recoveryWaitsForTheWheels() {
        getReady(2000);
        shooter.fire(2);
        run(0.02, 2000);   // feed starts with the wheels at speed
        run(0.24, 1800);   // first shot done, wheels sagging
        assertEquals(ShooterLogic.State.RECOVERING, shooter.getState());
        run(1.0, 1800);    // wheels stay slow
        assertEquals("never feed into a sagging wheel", ShooterLogic.State.RECOVERING, shooter.getState());
        run(0.2, 2000);
        assertTrue(shooter.ownsIndexer());
    }

    @Test public void minRecoveryAppliesEvenIfWheelsNeverSag() {
        getReady(2000);
        shooter.fire(2);
        run(0.22, 2000);
        assertEquals(ShooterLogic.State.RECOVERING, shooter.getState());
        run(0.04, 2000);
        assertEquals(ShooterLogic.State.RECOVERING, shooter.getState());
    }

    @Test public void continuousModeRunsOnceForAllPieces() {
        ShooterLogic.FEED_MODE = ShooterLogic.FeedMode.CONTINUOUS;
        getReady(2000);
        shooter.fire(3);
        run(0.02, 2000);
        assertTrue(shooter.ownsIndexer());
        run(0.84, 1900);            // 0.86 s in: still feeding (3 x 0.30 = 0.90)
        assertTrue(shooter.ownsIndexer());
        run(0.06, 1900);
        assertFalse(shooter.ownsIndexer());
        assertEquals(3, shooter.getShotsFired());
        assertFalse(shooter.isBusy());
    }

    @Test public void stopDropsEverything() {
        getReady(2000);
        shooter.fire(3);
        run(0.02, 2000);
        shooter.stop();
        run(0.02, 2000);
        assertEquals(ShooterLogic.State.IDLE, shooter.getState());
        assertEquals(0, shooter.getTargetRpm(), EPS);
        assertFalse(shooter.ownsIndexer());
        assertFalse(shooter.isBusy());
    }

    @Test public void extraFireDuringAVolleyAddsOn() {
        getReady(2000);
        shooter.fire(1);
        run(0.02, 2000);
        shooter.fire(1);
        assertEquals(2, shooter.getShotsQueued());
    }
}
