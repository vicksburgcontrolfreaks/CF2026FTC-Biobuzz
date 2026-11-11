package org.firstinspires.ftc.teamcode.geometry;

import org.junit.After;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/*
 * AimLead is the rate estimator behind turret lead. The fiddly parts are the
 * +/-180 wrap and the blind-arc jump, so those are what these check.
 */
public class AimLeadTest {

    private static final double DT = 0.02;   // a 50 Hz loop

    @After public void restoreDefaults() {
        AimLead.LEAD_SEC = 0.0;
        AimLead.SMOOTHING = 0.30;
        AimLead.RATE_JUMP_DEG = 20.0;
        AimLead.MAX_RATE_DEG_PER_SEC = 400.0;
    }

    /** Feeds a constant rate for long enough that the smoothing has converged. */
    private static AimLead spunAt(double degPerSec, int steps) {
        AimLead lead = new AimLead();
        double angle = 0;
        for (int i = 0; i < steps; i++) {
            lead.update(TurretGeometry.wrapDegrees(angle), i * DT);
            angle += degPerSec * DT;
        }
        return lead;
    }

    @Test public void firstSampleCannotKnowARate() {
        AimLead lead = new AimLead();
        assertEquals(0.0, lead.update(37.0, 0.0), 1e-9);
    }

    @Test public void convergesOnASteadyRate() {
        assertEquals(60.0, spunAt(60.0, 200).rateDegPerSec(), 0.5);
        assertEquals(-150.0, spunAt(-150.0, 200).rateDegPerSec(), 0.5);
    }

    /*
     * The one that matters. Sweeping through +180 makes the desired angle jump
     * to -180, and a naive subtraction reads that as -360 deg in one loop --
     * about -18000 deg/sec, pointing the wrong way. wrapDegrees has to absorb it.
     */
    @Test public void sweepingThroughTheWrapDoesNotInvertTheRate() {
        AimLead lead = new AimLead();
        double angle = 150;
        double worstSeen = Double.MAX_VALUE;   // tracking the LOWEST rate seen
        for (int i = 0; i < 400; i++) {
            lead.update(TurretGeometry.wrapDegrees(angle), i * DT);
            angle += 60 * DT;
            if (i > 50) worstSeen = Math.min(worstSeen, lead.rateDegPerSec());
        }
        assertTrue("rate went negative through the wrap: " + worstSeen, worstSeen > 0);
        assertEquals(60.0, lead.rateDegPerSec(), 0.5);
    }

    /*
     * A blind-arc flip is the desired angle moving ~180 deg in ONE loop. That is
     * not the target moving and must not become a rate -- the lead would fling
     * the turret at a target that never went anywhere.
     */
    @Test public void aBlindArcFlipIsDroppedNotDifferentiated() {
        AimLead lead = spunAt(60.0, 200);
        double before = lead.rateDegPerSec();

        lead.update(TurretGeometry.wrapDegrees(before * DT * 200 + 175), 200 * DT);
        double afterFlip = lead.update(-175, 201 * DT);

        assertTrue("the flip should have been dropped", lead.droppedLastSample());
        assertEquals("estimate should be held, not recomputed", before, afterFlip, 15.0);
    }

    @Test public void aStoppedTargetDecaysToZero() {
        AimLead lead = spunAt(90.0, 200);
        assertTrue(lead.rateDegPerSec() > 50);
        for (int i = 0; i < 300; i++) {
            lead.update(0.0, (200 + i) * DT);
        }
        assertEquals(0.0, lead.rateDegPerSec(), 0.5);
    }

    @Test public void leadIsOffWhenLeadSecIsZero() {
        AimLead lead = spunAt(120.0, 200);
        AimLead.LEAD_SEC = 0.0;
        assertEquals(0.0, lead.leadDeg(), 1e-9);
        AimLead.LEAD_SEC = 0.25;
        assertEquals(0.25 * lead.rateDegPerSec(), lead.leadDeg(), 1e-9);
    }

    @Test public void oneWildSampleCannotDominate() {
        AimLead.RATE_JUMP_DEG = 1000;          // let the sample through
        AimLead.MAX_RATE_DEG_PER_SEC = 400;
        AimLead lead = new AimLead();
        lead.update(0, 0);
        lead.update(19, DT);                   // 950 deg/sec, clamped to 400
        assertTrue(Math.abs(lead.rateDegPerSec()) <= 400 * AimLead.SMOOTHING + 1e-6);
    }

    @Test public void aRepeatedTimestampIsIgnored() {
        AimLead lead = spunAt(60.0, 200);
        double before = lead.rateDegPerSec();
        lead.update(999, 199 * DT);            // same clock as the last sample
        assertTrue(lead.droppedLastSample());
        assertEquals(before, lead.rateDegPerSec(), 1e-9);
    }
}
