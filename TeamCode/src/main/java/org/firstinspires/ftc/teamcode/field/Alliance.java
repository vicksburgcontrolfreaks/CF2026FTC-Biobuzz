package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/*
 * Which alliance we're on. We only ever type in RED positions; this turns
 * them into the matching position for whichever alliance we really are.
 *
 * The BIOBUZZ field is the same when spun 180 degrees around its center
 * (see FieldConstants), so blue = red spun 180 degrees.
 */
public enum Alliance {
    RED,
    BLUE;

    // Turn a red-side pose into the pose for this alliance.
    public Pose map(Pose redPose) {
        return this == RED ? redPose : FieldConstants.rotate180(redPose);
    }

    /**
     * The field heading (radians) that points straight AWAY from this alliance's
     * drivers. Red drives from the red wall (x = 0) looking toward +x, so 0;
     * blue drives from the blue wall (x = 144) looking toward -x, so 180 deg.
     * Field-centric TeleOp uses this as "stick forward".
     */
    public double awayFromDriversHeading() {
        return this == RED ? 0.0 : Math.PI;
    }
}
