package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Auton.Waypoints;
import org.firstinspires.ftc.teamcode.geometry.WallFix;

/**
 * The four spots whose coordinates are known with no tape measure: robot facing
 * the OTHER alliance, side touching the audience or scoring wall, back edge on
 * the tile seam nearest its own alliance wall (x = 24 for red, x = 120 for blue).
 *
 * Shared by Tag Pose Test (as the truth to compare a fix against) and Shot Map
 * Test (as the pose to start from, and to re-sync to by parking back on it).
 *
 * THE MIRRORED PAIRS (Shane, 2026-11-13): red audience <-> blue scoring and red
 * scoring <-> blue audience are exact 180 deg rotations of each other, so the
 * camera sees the same picture from both. An error that lives on the ROBOT
 * (camera yaw/left/pitch, turret zero, lens) FLIPS SIGN in field coordinates
 * between a pair; an error in the FIELD MODEL (HiveTags, the blue hive placed by
 * symmetry, our field's build tolerance) does not.
 *
 * Blue is built with Alliance.map, never typed in, so all four stay in step with
 * Waypoints if the robot size ever changes.
 */
public enum WallStart {
    // Facing the other alliance, the audience wall is on the robot's RIGHT and
    // the scoring wall on its LEFT -- for both alliances.
    RED_AUDIENCE("red, audience wall", Alliance.RED, Waypoints.GARDEN_EDGE_START, WallFix.Face.RIGHT),
    RED_SCORING("red, scoring wall", Alliance.RED, Waypoints.FAR_EDGE_START, WallFix.Face.LEFT),
    BLUE_SCORING("blue, scoring wall", Alliance.BLUE, Waypoints.GARDEN_EDGE_START, WallFix.Face.RIGHT),
    BLUE_AUDIENCE("blue, audience wall", Alliance.BLUE, Waypoints.FAR_EDGE_START, WallFix.Face.LEFT);

    public final String label;
    public final Alliance alliance;
    public final Pose pose;
    /** Which side of the robot touches the wall here. */
    public final WallFix.Face face;

    WallStart(String label, Alliance alliance, Pose redPose, WallFix.Face face) {
        this.label = label;
        this.alliance = alliance;
        this.pose = alliance.map(redPose);
        this.face = face;
    }

    /** Heading in degrees, normalised to (-180, 180] so blue reads 180. */
    public double headingDeg() {
        double h = pose.getHeading();
        return Math.toDegrees(Math.atan2(Math.sin(h), Math.cos(h)));
    }

    /** The next one, wrapping round. For a "cycle with one button" control. */
    public WallStart next() {
        return values()[(ordinal() + 1) % values().length];
    }

    @Override
    public String toString() {
        return label;
    }
}
