package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutonConstants {
    // Blue poses
    public static final Pose blueLongStart = new Pose(88, 8, Math.toRadians(270)); // Start position from long shot
    public static final Pose blueLongLoad = new Pose(118, 13, Math.toRadians(0)); // Load position on long shot
    public static final Pose blueLongScore = new Pose(82, 17, Math.toRadians(305)); // Scoring position on long shot
    public static final Pose blueShortStart = new Pose(109, 133, Math.toRadians(0)); // Start position from short shot
    public static final Pose blueSpike1 = new Pose(102, 83, Math.toRadians(0)); // G-P-P
    public static final Pose blueSpike2 = new Pose(102, 59, Math.toRadians(0)); // P-G-P
    public static final Pose blueSpike3 = new Pose(102, 35, Math.toRadians(0)); // P-P-G
    public static final Pose blueSpike1Post = new Pose(122, 83, Math.toRadians(0)); // G-P-P post
    public static final Pose blueSpike2Post = new Pose(122, 59, Math.toRadians(0)); // P-G-P post
    public static final Pose blueSpike3Post = new Pose(122, 35, Math.toRadians(0)); // P-P-G post
    public static final Pose blueShortScore = new Pose(86, 83, Math.toRadians(320)); // Scoring position on short shot

    // Red poses (x=72 mirror, heading + 180° normalized)
    public static final Pose redLongStart = new Pose(54, 11, Math.toRadians(120)); // 300° + 180° = 480° - 360° = 120°
    public static final Pose redLongLoad = new Pose(26, 13, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redLongScore = new Pose(62, 17, Math.toRadians(125)); // 305° + 180° = 485° - 360° = 125°
    public static final Pose redShortStart = new Pose(35, 133, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike1 = new Pose(42, 83, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike2 = new Pose(42, 59, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike3 = new Pose(42, 35, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike1Post = new Pose(22, 83, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike2Post = new Pose(22, 59, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike3Post = new Pose(22, 35, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redShortScore = new Pose(58, 83, Math.toRadians(140)); // 320° + 180° = 500° - 360° = 140°

    // Blue paths
    public static Path blueLongLoadPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueLongLoad));
        path.setLinearHeadingInterpolation(start.getHeading(), blueLongLoad.getHeading());
        return path;
    }

    public static Path blueLongScorePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueLongScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueLongScore.getHeading());
        return path;
    }

    public static Path blueSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike1.getHeading());
        return path;
    }

    public static Path blueStartToSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike1.getHeading());
        return path;
    }

    public static Path blueStartToSpikeTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike2));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike2.getHeading());
        return path;
    }

    public static Path blueStartToSpikeThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike3));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike3.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    // Red paths
    public static Path redLongLoadPath(Pose start) {
        Path path = new Path(new BezierLine(start, redLongLoad));
        path.setLinearHeadingInterpolation(start.getHeading(), redLongLoad.getHeading());
        return path;
    }

    public static Path redLongScorePath(Pose start) {
        Path path = new Path(new BezierLine(start, redLongScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redLongScore.getHeading());
        return path;
    }

    public static Path redStartToSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike1.getHeading());
        return path;
    }

    public static Path redStartToSpikeTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike2));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike2.getHeading());
        return path;
    }

    public static Path redStartToSpikeThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike3));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike3.getHeading());
        return path;
    }

    public static Path redSpikeToScoreOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }

    public static Path redSpikeToScoreTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }

    public static Path redSpikeToScoreThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }
    private Path scorePreload;

}