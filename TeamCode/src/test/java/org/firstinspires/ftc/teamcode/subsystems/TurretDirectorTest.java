package org.firstinspires.ftc.teamcode.subsystems;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/*
 * The part of TurretDirector that can be tested on a laptop: the camera-versus-
 * odometry disagreement that becomes the trim. Everything else in that class
 * needs a Turret and real AprilTagDetections.
 *
 * This is the piece worth testing anyway, because it is where a sign error would
 * hide. A trim with the wrong sign does not fail loudly -- it doubles the error
 * it was meant to cancel, which on the robot looks exactly like bad geometry.
 */
public class TurretDirectorTest {

    private static final double EPS = 1e-9;

    /** No disagreement: the camera says the target is exactly where the pose says. */
    @Test public void agreementMeansNoTrim() {
        // Turret at 30, camera sees the target 5 deg off its nose, so the camera
        // puts the target at 35. Odometry agrees it is at 35.
        assertEquals(0.0, TurretDirector.trimFrom(30, 5, 1.0, 0, 35), EPS);
    }

    /** The camera is the truth, so the trim points toward what the camera says. */
    @Test public void trimIsHowFarTheCameraDisagrees() {
        // Camera puts the target at 35, odometry thinks 32: the pose is 3 deg out
        // and the aim needs +3.
        assertEquals(3.0, TurretDirector.trimFrom(30, 5, 1.0, 0, 32), EPS);
        // The other way round.
        assertEquals(-3.0, TurretDirector.trimFrom(30, 5, 1.0, 0, 38), EPS);
    }

    /**
     * The camera is on the TURRET, so a bearing is relative to where the turret
     * is pointing. The same bearing from a different turret angle is a different
     * place on the field, and the trim has to reflect that.
     */
    @Test public void theBearingIsRelativeToTheTurretNotTheRobot() {
        double fromThirty = TurretDirector.trimFrom(30, 5, 1.0, 0, 35);
        double fromSixty = TurretDirector.trimFrom(60, 5, 1.0, 0, 35);
        assertEquals(0.0, fromThirty, EPS);
        assertEquals(30.0, fromSixty, EPS);
    }

    /** BEARING_SIGN flips the camera's convention, so it must flip the trim too. */
    @Test public void bearingSignIsHonoured() {
        assertEquals(10.0, TurretDirector.trimFrom(30, 5, 1.0, 0, 25), EPS);
        assertEquals(0.0, TurretDirector.trimFrom(30, 5, -1.0, 0, 25), EPS);
    }

    /** AIM_BIAS_DEG is part of where the camera says to point, so it rides along. */
    @Test public void aimBiasIsIncluded() {
        assertEquals(2.0, TurretDirector.trimFrom(30, 5, 1.0, 2.0, 35), EPS);
    }

    /*
     * The wrap. A turret near +180 and a desired angle near -180 are nearly the
     * same direction, and a raw subtraction calls that a 350 deg disagreement --
     * which MAX_TRIM_DEG would then reject, silently throwing away a perfectly
     * good fix, or worse, apply.
     */
    @Test public void disagreementAcrossTheWrapStaysSmall() {
        double trim = TurretDirector.trimFrom(175, 10, 1.0, 0, -172);
        assertEquals(-3.0, trim, EPS);
        assertTrue("wrap not applied: " + trim, Math.abs(trim) < 180);
    }

    @Test public void anExactlyOppositeTargetDoesNotBlowUp() {
        double trim = TurretDirector.trimFrom(0, 0, 1.0, 0, 180);
        assertTrue("should wrap into +/-180, got " + trim, Math.abs(trim) <= 180);
    }

    /*
     * A trim bigger than MAX_TRIM_DEG is dropped by update(), so the constant has
     * to be big enough to cover a believable pose error and small enough to catch
     * a wrong-cell sighting. The two cells of a hive are about 18.8 in. apart, so
     * mistaking one for the other at 60 in. is roughly 18 deg -- outside the cap,
     * which is the point.
     */
    @Test public void theCapSitsBelowAWrongCellSighting() {
        double wrongCellDeg = Math.toDegrees(Math.atan2(18.8, 60));
        assertTrue("a wrong-cell sighting (" + wrongCellDeg + " deg) should exceed MAX_TRIM_DEG",
                wrongCellDeg > TurretDirector.MAX_TRIM_DEG);
        // And a few degrees of ordinary pose error should NOT be thrown away.
        assertTrue(5.0 < TurretDirector.MAX_TRIM_DEG);
    }
}
