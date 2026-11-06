package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.WhichRobot;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-48.3)
            .lateralZeroPowerAcceleration(-85.5)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.00001, 0.6, 0.01))
            .mass(7.7);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);



    /*
     * DRIVE SETUP: ONE PER ROBOT
     * 5618 and 6494 have different drivetrains, so the motor directions differ
     * (6494's rear wheels spin the opposite way). WhichRobot reads the active
     * config name and driveConstants() hands back the right robot's setup.
     * Everything that drives goes through createFollower() below, so TeleOp,
     * autos, Tuning, and tests all pick the right one automatically.
     *
     * To fix a wheel's direction, run "Drive Motor Direction Test" on THAT robot
     * and change only THAT robot's block.
     *
     * Note: xVelocity/yVelocity (and the followerConstants above) were tuned on
     * one robot and are shared. If the gearing differs, re-tune and move them
     * into the per-robot blocks.
     */
    public static MecanumConstants driveConstants() {
        switch (WhichRobot.get()) {
            case R5618:
                // Per-wheel test on 5618, 2026-11-05: every wheel spun backwards
                // with the old settings, so all four were flipped to these.
                return sharedDrive()
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            case R6494:
            default:
                // Per-wheel test on 6494, 2026-11-05: with 5618's settings the
                // fronts were right and both rears spun backwards, so the rears
                // are flipped relative to 5618.
                return sharedDrive()
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Sets this robot's drive directions on raw motors, for code that drives
     * without Pedro (RobotHardware, sanity tests). The ONLY way anything outside
     * Pedro should set a drive direction -- never hardcode them.
     */
    public static void applyDriveDirections(DcMotorSimple lf, DcMotorSimple lr,
                                            DcMotorSimple rf, DcMotorSimple rr) {
        MecanumConstants drive = driveConstants();
        lf.setDirection(drive.leftFrontMotorDirection);
        lr.setDirection(drive.leftRearMotorDirection);
        rf.setDirection(drive.rightFrontMotorDirection);
        rr.setDirection(drive.rightRearMotorDirection);
    }

    /** The parts of the drive setup that are the same on both robots. */
    private static MecanumConstants sharedDrive() {
        return new MecanumConstants()
                .maxPower(1)
                .xVelocity(53.38354732)
                .yVelocity(40.318533829)
                .rightFrontMotorName("rf") //EH1
                .rightRearMotorName("rr") //EH0
                .leftRearMotorName("lr") //CH0
                .leftFrontMotorName("lf"); //CH1 - also carries the turret encoder
    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.89) //175mm 6.89in
            .strafePodX(0) // 0
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //CH I2C Bus 0
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants())
                .build();
    }
}



