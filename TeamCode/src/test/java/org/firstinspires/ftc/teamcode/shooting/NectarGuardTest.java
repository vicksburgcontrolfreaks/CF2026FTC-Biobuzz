package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.shooting.NectarGuard.Gap;
import org.firstinspires.ftc.teamcode.shooting.NectarGuard.State;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/*
 * NectarGuard on the laptop:  gradlew :TeamCode:testDebugUnitTest
 * Uses the default thresholds: TRIP 1.51 A, BLIND after 0.5 s under 0.05 A,
 * JAM 400 RPM (armed below 200).
 */
public class NectarGuardTest {

    private static final double OFF = Double.NaN;   // flywheel off
    private static final double EMPTY_AMPS = 0.6;
    private static final double NECTAR_AMPS = 2.1;

    private NectarGuard g;
    private double t;

    @Before
    public void setUp() {
        NectarGuard.TRIP_AMPS = 1.51;
        NectarGuard.BLANK_SEC = 0.0;
        NectarGuard.BLIND_AFTER_SEC = 0.5;
        NectarGuard.MIN_ALIVE_AMPS = 0.05;
        NectarGuard.JAM_DIP_RPM = 400;
        g = new NectarGuard();
        t = 0;
    }

    /** One 20 ms loop. */
    private void step(boolean forward, double amps, boolean shot, double drop) {
        t += 0.02;
        g.update(t, forward, amps, shot, drop);
    }

    private void run(int loops, boolean forward, double amps, double drop) {
        for (int i = 0; i < loops; i++) step(forward, amps, false, drop);
    }

    @Test
    public void startsAtPollen() {
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void pollenLoadNeverTrips() {
        run(100, true, EMPTY_AMPS, 20);
        assertEquals(Gap.POLLEN, g.gap());
        assertEquals(0, g.getTrips());
    }

    @Test
    public void nectarLoadOpensToNectarGap_andStaysUntilItsShot() {
        run(10, true, EMPTY_AMPS, 20);
        step(true, NECTAR_AMPS, false, 20);
        assertEquals(Gap.NECTAR, g.gap());
        assertEquals(1, g.getTrips());

        // Ball leaves the indexer, rolls down the ramp: still nectar.
        run(40, true, EMPTY_AMPS, 20);
        run(20, false, 0, 20);
        assertEquals(Gap.NECTAR, g.gap());

        // Its shot -> pollen again.
        step(false, 0, true, 150);
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void nectarDetectedWithFlywheelOff_staysNectarUntilAShot() {
        step(true, NECTAR_AMPS, false, OFF);
        run(500, false, 0, OFF);   // sits on the ramp for 10 s
        assertEquals(Gap.NECTAR, g.gap());
    }

    @Test
    public void deadCurrentReading_failsSafeToNectar() {
        run(30, true, 0.0, 20);   // 0.6 s forward, reads nothing
        assertEquals(State.BLIND, g.getState());
        assertEquals(Gap.NECTAR, g.gap());
    }

    @Test
    public void blind_recoversWhenCurrentReadsAgain() {
        run(30, true, 0.0, 20);
        run(5, false, 0, 20);
        run(5, true, EMPTY_AMPS, 20);
        assertEquals(State.CLEAR, g.getState());
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void shortRunsDoNotCountAsBlind() {
        for (int i = 0; i < 10; i++) {
            run(10, true, 0.0, 20);   // 0.2 s bursts
            run(5, false, 0, 20);
        }
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void requestedNectar_holdsUntilNextShot() {
        g.requestNectar();
        assertEquals(Gap.NECTAR, g.gap());
        run(50, true, EMPTY_AMPS, 20);
        assertEquals(Gap.NECTAR, g.gap());
        step(false, 0, true, 120);
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void spinUp_doesNotTripTheBackstop() {
        // Wheels from 0 to 1300: drop goes 1300 -> 0.
        for (double drop = 1300; drop >= 0; drop -= 20) step(false, 0, false, drop);
        assertFalse(g.isFault());
    }

    @Test
    public void bigDipAtPollenGap_faults_andOpensTheGap() {
        run(10, false, 0, 10);    // at speed -> armed
        step(true, EMPTY_AMPS, false, 450);
        assertTrue(g.isFault());
        assertEquals(Gap.OPEN, g.gap());
        // Latched: nothing un-faults it but reset().
        run(100, false, 0, 0);
        step(false, 0, true, 0);
        assertTrue(g.isFault());
        g.reset();
        assertFalse(g.isFault());
        assertEquals(Gap.POLLEN, g.gap());
    }

    @Test
    public void normalPollenDip_isNotAJam() {
        run(10, false, 0, 10);
        step(true, EMPTY_AMPS, true, 130);
        assertFalse(g.isFault());
    }

    @Test
    public void bigDipAtNectarGap_isNotAJam() {
        run(10, false, 0, 10);
        step(true, NECTAR_AMPS, false, 10);   // nectar detected -> nectar gap
        step(false, 0, false, 600);           // a nectar shot may dip a lot
        assertFalse(g.isFault());
    }

    @Test
    public void recoveryAfterANectarShot_doesNotTripTheBackstop() {
        run(10, false, 0, 10);
        step(true, NECTAR_AMPS, false, 10);
        step(false, 0, true, 600);            // nectar shot: guard clears to pollen
        assertEquals(Gap.POLLEN, g.gap());
        // Wheels still way down while recovering, now at the pollen gap.
        for (double drop = 600; drop >= 0; drop -= 20) step(false, 0, false, drop);
        assertFalse(g.isFault());
    }

    @Test
    public void tripCountsOncePerNectar() {
        run(5, true, NECTAR_AMPS, 20);   // several loops above threshold
        assertEquals(1, g.getTrips());
    }

    @Test
    public void lastRunPeak_isKeptForCalibration() {
        run(5, true, 1.2, 20);
        run(2, false, 0, 20);
        assertEquals(1.2, g.getLastRunPeakAmps(), 1e-9);
    }
}
