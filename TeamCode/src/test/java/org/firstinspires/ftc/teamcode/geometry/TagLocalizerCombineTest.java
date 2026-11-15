package org.firstinspires.ftc.teamcode.geometry;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

/*
 * TagLocalizer.combine(): several tag fixes from one frame -> one fix.
 * Fixes are made up directly (no camera math); TagLocalizerTest covers that.
 */
public class TagLocalizerCombineTest {

    private static final double EPS = 1e-9;

    private static TagLocalizer.Fix fix(int id, double x, double y, double headingDeg, double range) {
        return fix(id, x, y, headingDeg, range, 0);
    }

    /** With a bearing, for the tests that care how combine() carries it. */
    private static TagLocalizer.Fix fix(int id, double x, double y, double headingDeg,
                                        double range, double bearingDeg) {
        return new TagLocalizer.Fix(new Pose(x, y, Math.toRadians(headingDeg)), null, id, range,
                bearingDeg, 1234L);
    }

    private static double headingDeg(TagLocalizer.Fix f) {
        return TurretGeometry.wrapDegrees(Math.toDegrees(f.robot.getHeading()));
    }

    @Test public void nothingInNothingOut() {
        assertNull(TagLocalizer.combine(Collections.<TagLocalizer.Fix>emptyList()));
    }

    @Test public void oneTagPassesThrough() {
        TagLocalizer.Fix only = fix(31, 50, 40, 90, 30);
        assertSame(only, TagLocalizer.combine(Collections.singletonList(only)));
    }

    @Test public void fourAgreeingTagsAverage() {
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 50.0, 40.0, 90, 30),
                fix(31, 50.4, 40.2, 91, 32),
                fix(32, 49.8, 39.6, 89, 34),
                fix(33, 50.2, 40.2, 90, 36)));
        assertEquals(4, f.tagsUsed);
        assertEquals(50.1, f.robot.getX(), EPS);
        assertEquals(40.0, f.robot.getY(), EPS);
        assertEquals(90.0, headingDeg(f), EPS);
        assertEquals(33.0, f.tagRange, EPS);
        assertEquals(1234L, f.frameNanos);
    }

    @Test public void bearingTakesTheWorstTagNotTheAverage() {
        // Range averages, bearing does NOT: one tag out at the frame edge is
        // enough to bias the solve, so the combined fix has to be judged by it.
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 50.0, 40.0, 90, 30, 1.0),
                fix(31, 50.4, 40.2, 91, 32, 2.0),
                fix(32, 49.8, 39.6, 89, 34, 24.0),
                fix(33, 50.2, 40.2, 90, 36, 3.0)));
        assertEquals(24.0, f.tagBearingDeg, EPS);
    }

    @Test public void aFlippedTagIsDropped() {
        // Tag 32 "flipped" to a solution 8 in. away. A plain average would land
        // 2 in. off; combine() should ignore it entirely.
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 50, 40, 90, 30),
                fix(31, 50, 40, 90, 30),
                fix(32, 58, 40, 90, 30),
                fix(33, 50, 40, 90, 30)));
        assertEquals(3, f.tagsUsed);
        assertEquals(50, f.robot.getX(), EPS);
        assertEquals(0, f.spread, EPS);
    }

    @Test public void aTagWithAWildHeadingIsDropped() {
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 50, 40, 90, 30),
                fix(31, 50, 40, 90, 30),
                fix(32, 50, 40, 110, 30)));
        assertEquals(2, f.tagsUsed);
        assertEquals(90, headingDeg(f), EPS);
    }

    @Test public void headingsAverageTheShortWayRound() {
        // 178 and -178 are 4 deg apart, average 180 -- not 0.
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 50, 40, 178, 30),
                fix(31, 50, 40, -178, 30)));
        assertEquals(2, f.tagsUsed);
        assertEquals(180, Math.abs(headingDeg(f)), EPS);
    }

    @Test public void twoTagsThatDisagreeGiveNothing() {
        // 10 in. apart: the median sits 5 in. from each, past OUTLIER_DISTANCE.
        assertNull(TagLocalizer.combine(Arrays.asList(
                fix(30, 50, 40, 90, 30),
                fix(31, 60, 40, 90, 30))));
    }

    @Test public void spreadAndRepresentativeTag() {
        TagLocalizer.Fix f = TagLocalizer.combine(Arrays.asList(
                fix(30, 49, 40, 90, 30),
                fix(31, 50, 40, 90, 30),
                fix(32, 51, 40, 90, 30)));
        assertEquals(1.0, f.spread, EPS);
        assertEquals(31, f.tagId);   // the one sitting on the average
    }
}
