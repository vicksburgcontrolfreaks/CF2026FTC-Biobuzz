package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Laptop tests for the wall re-anchor guards. Every rule in WallReanchor is here,
 * plus the ORDER they fire in -- the order is a real decision, not an accident,
 * because the first failure is what the driver gets told.
 */
public class WallReanchorTest {

    /** Somewhere legal and boring: on the audience wall, nowhere near a flower. */
    private static final Pose GOOD_SPOT = new Pose(33, 9, 0);

    @Before
    public void resetTunables() {
        // These are public static and Panels-editable, so a test that changed one
        // could silently break the next. Put them back.
        WallReanchor.MAX_SPEED = 2.0;
        WallReanchor.MAX_TURN_RATE_DEG = 5.0;
        WallReanchor.MAX_SPREAD = 3.0;
        WallReanchor.MAX_INCHES_PER_DEG = 3.0;
        WallReanchor.MAX_SNAP_DEG = 6.0;
        WallReanchor.FIELD_MARGIN = 2.0;
    }

    /** A fix that passes everything, so each test can spoil exactly one thing. */
    private WallFix.Result good() {
        return fix(GOOD_SPOT, 0.0, 4, 0.5, 1.0);
    }

    private WallFix.Result fix(Pose robot, double snapDeg, int tagsUsed,
                               double spread, double inchesPerDeg) {
        return new WallFix.Result(robot, "ok", WallFix.Wall.AUDIENCE,
                snapDeg, tagsUsed, spread, inchesPerDeg);
    }

    private WallFix.Result refused() {
        return new WallFix.Result(null, "sight line too shallow",
                WallFix.Wall.AUDIENCE, 0, 0, 0, 0);
    }

    private WallReanchor.Verdict check(WallFix.Result f) {
        return WallReanchor.check(f, 0.0, 0.0, true);
    }

    @Test
    public void aCleanFixParkedAndSettledIsAccepted() {
        assertEquals(WallReanchor.Verdict.ACCEPT, check(good()));
        assertTrue(WallReanchor.Verdict.ACCEPT.accepted());
    }

    @Test
    public void onlyAcceptCountsAsAccepted() {
        for (WallReanchor.Verdict v : WallReanchor.Verdict.values()) {
            assertEquals(v == WallReanchor.Verdict.ACCEPT, v.accepted());
        }
    }

    @Test
    public void nullOrRefusedFixIsNoFix() {
        assertEquals(WallReanchor.Verdict.NO_FIX, check(null));
        assertEquals(WallReanchor.Verdict.NO_FIX, check(refused()));
    }

    @Test
    public void aRollingRobotIsRefused() {
        assertEquals(WallReanchor.Verdict.ROBOT_MOVING,
                WallReanchor.check(good(), 5.0, 0.0, true));
        // Sign must not matter -- reversing into the wall is the normal case.
        assertEquals(WallReanchor.Verdict.ROBOT_MOVING,
                WallReanchor.check(good(), -5.0, 0.0, true));
    }

    @Test
    public void aLittleNoiseWhileParkedStillAccepts() {
        // The reason MAX_SPEED is not zero: demanding exact zero means the button
        // never works on a real robot.
        assertEquals(WallReanchor.Verdict.ACCEPT,
                WallReanchor.check(good(), 1.0, 2.0, true));
    }

    @Test
    public void aSpinningRobotIsRefused() {
        assertEquals(WallReanchor.Verdict.TURRET_MOVING,
                WallReanchor.check(good(), 0.0, 30.0, true));
        assertEquals(WallReanchor.Verdict.TURRET_MOVING,
                WallReanchor.check(good(), 0.0, -30.0, true));
    }

    @Test
    public void anUnsettledTurretIsRefused() {
        assertEquals(WallReanchor.Verdict.TURRET_MOVING,
                WallReanchor.check(good(), 0.0, 0.0, false));
    }

    @Test
    public void anAnswerOffTheFieldIsRefused() {
        assertEquals(WallReanchor.Verdict.OFF_FIELD, check(fix(
                new Pose(-5, 60, 0), 0, 4, 0.5, 1.0)));
        assertEquals(WallReanchor.Verdict.OFF_FIELD, check(fix(
                new Pose(FieldConstants.FIELD_SIZE + 5, 60, 0), 0, 4, 0.5, 1.0)));
        assertEquals(WallReanchor.Verdict.OFF_FIELD, check(fix(
                new Pose(60, -1, 0), 0, 4, 0.5, 1.0)));
    }

    @Test
    public void theFieldMarginIsEnforcedNotJustTheWall() {
        // 1 in inside the wall is ON the field but inside the 2 in margin.
        assertEquals(WallReanchor.Verdict.OFF_FIELD, check(fix(
                new Pose(1, 60, 0), 0, 4, 0.5, 1.0)));
        // 3 in is outside the margin and fine.
        assertEquals(WallReanchor.Verdict.ACCEPT, check(fix(
                new Pose(3, 60, 0), 0, 4, 0.5, 1.0)));
    }

    @Test
    public void anAnswerInsideAFlowerIsRefused() {
        for (org.firstinspires.ftc.teamcode.field.Zone flower : FieldConstants.FLOWERS) {
            Pose inside = new Pose(flower.centerX(), flower.centerY(), 0);
            assertEquals("flower " + flower.name,
                    WallReanchor.Verdict.IN_FLOWER, check(fix(inside, 0, 4, 0.5, 1.0)));
        }
    }

    @Test
    public void tagsThatDisagreeAreRefused() {
        assertEquals(WallReanchor.Verdict.TAGS_DISAGREE,
                check(fix(GOOD_SPOT, 0, 4, 9.0, 1.0)));
    }

    @Test
    public void spreadExactlyAtTheLimitIsStillAccepted() {
        // The rule is "more than", so the boundary belongs to ACCEPT. Pinned
        // because flipping > to >= here would reject half the good frames.
        assertEquals(WallReanchor.Verdict.ACCEPT,
                check(fix(GOOD_SPOT, 0, 4, WallReanchor.MAX_SPREAD, 1.0)));
    }

    @Test
    public void anObliqueSightLineIsRefused() {
        assertEquals(WallReanchor.Verdict.TOO_OBLIQUE,
                check(fix(GOOD_SPOT, 0, 4, 0.5, 12.0)));
    }

    @Test
    public void aBigHeadingSnapIsRefused() {
        assertEquals(WallReanchor.Verdict.SNAP_TOO_BIG,
                check(fix(GOOD_SPOT, 8.0, 4, 0.5, 1.0)));
        // Either direction. A snap is a correction, and its sign is arbitrary.
        assertEquals(WallReanchor.Verdict.SNAP_TOO_BIG,
                check(fix(GOOD_SPOT, -8.0, 4, 0.5, 1.0)));
    }

    @Test
    public void aSnapInsideTheLimitIsFine() {
        assertEquals(WallReanchor.Verdict.ACCEPT,
                check(fix(GOOD_SPOT, 5.0, 4, 0.5, 1.0)));
    }

    // ---- rule ORDER ----------------------------------------------------------
    // The driver sees ONE reason, so which one wins matters.

    @Test
    public void motionIsReportedBeforeGeometry() {
        // A rolling robot with a terrible fix should say ROBOT_MOVING: stop first,
        // then the geometry complaint might not even happen.
        WallFix.Result bad = fix(new Pose(-20, -20, 0), 20, 1, 50, 50);
        assertEquals(WallReanchor.Verdict.ROBOT_MOVING,
                WallReanchor.check(bad, 10.0, 0.0, true));
    }

    @Test
    public void noFixBeatsEverything() {
        // Nothing else is meaningful when there is no answer to judge.
        assertEquals(WallReanchor.Verdict.NO_FIX,
                WallReanchor.check(refused(), 10.0, 10.0, false));
    }

    @Test
    public void physicallyImpossibleBeatsMerelyNoisy() {
        // Off the field AND a big spread -> OFF_FIELD, because that is the one a
        // driver cannot catch by eye in a match.
        assertEquals(WallReanchor.Verdict.OFF_FIELD,
                check(fix(new Pose(-5, 60, 0), 0, 4, 99.0, 1.0)));
    }

    // ---- the driver-facing string --------------------------------------------

    @Test
    public void describeCarriesTheNumberThatCausedIt() {
        WallFix.Result f = fix(GOOD_SPOT, 0, 4, 9.0, 1.0);
        String s = WallReanchor.describe(WallReanchor.Verdict.TAGS_DISAGREE, f);
        assertTrue(s, s.contains("TAGS_DISAGREE"));
        assertTrue("should quote the spread: " + s, s.contains("9.0"));
    }

    @Test
    public void describeOnARefusalQuotesWallFixsOwnReason() {
        String s = WallReanchor.describe(WallReanchor.Verdict.NO_FIX, refused());
        assertTrue(s, s.contains("sight line too shallow"));
    }

    @Test
    public void describeSurvivesNulls() {
        assertEquals("--", WallReanchor.describe(null, null));
        assertFalse(WallReanchor.describe(WallReanchor.Verdict.ACCEPT, null).isEmpty());
    }
}
