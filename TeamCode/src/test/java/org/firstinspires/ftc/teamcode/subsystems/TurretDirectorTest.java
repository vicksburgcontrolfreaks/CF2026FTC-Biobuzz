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
     * to be big enough to cover a real pose error and small enough to reject
     * nonsense.
     *
     * THIS TEST USED TO ASSERT THE OPPOSITE END, 2026-11-13. It required the cap
     * to sit BELOW a wrong-cell sighting (the hive's cells are ~18.8 in apart, so
     * ~18 deg at 60 in), which forced the cap under 18 and meant any drift past
     * that permanently dropped odometry aiming -- permanently, because the pose is
     * only re-trusted when the camera AGREES, which a drifted pose never does.
     *
     * The wrong-cell guard belongs to TurretAim, not here: it only averages tags
     * passing isTargetTag(), and acceptAnyHive is false in match code, so the
     * bearing behind this trim cannot come from the other cell. See MAX_TRIM_DEG.
     */
    @Test public void theCapAbsorbsRealDriftWithoutBelievingNonsense() {
        // A wrong-cell sighting is ~18 deg. It must now be ABSORBED, not rejected,
        // because tag-ID filtering is what prevents it -- and because 18 deg of
        // genuine pose drift has to be correctable.
        double wrongCellDeg = Math.toDegrees(Math.atan2(18.8, 60));
        assertTrue("drift of " + wrongCellDeg + " deg must be correctable, not dropped",
                wrongCellDeg < TurretDirector.MAX_TRIM_DEG);

        // Ordinary pose error, obviously kept.
        assertTrue(5.0 < TurretDirector.MAX_TRIM_DEG);

        // But still bounded: a turret angle wrong by a quadrant, or a garbage
        // frame, is not drift and must not be quietly applied to the aim.
        assertTrue("90 deg is not drift, it is a fault", 90.0 > TurretDirector.MAX_TRIM_DEG);
    }

    /*
     * The property that makes raising the cap safe: a single bad frame moves the
     * aim by a fraction of its error, not all of it. At TRIM_SMOOTHING 0.15 even a
     * frame at the new cap shifts the trim by under 7 deg, and the good frames
     * after it pull straight back.
     */
    @Test public void smoothingKeepsOneBadFrameFromMovingTheAimFar() {
        double worstStep = TurretDirector.MAX_TRIM_DEG * TurretDirector.TRIM_SMOOTHING;
        assertTrue("one frame can move the trim " + worstStep + " deg", worstStep < 10.0);
    }
}
