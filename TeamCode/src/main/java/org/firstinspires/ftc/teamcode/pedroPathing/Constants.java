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
    /*
     * PATH-FOLLOWING TUNING: ONE PER ROBOT.
     * Measured with Tuning > Automatic on a FRESH battery (a low battery read
     * ~15% slower on 6494). The PIDF gains are still the 2025 port values until
     * the Manual tuners are run. mass is NOT measured on either robot yet.
     */
    public static FollowerConstants followerConstants() {
        switch (WhichRobot.get()) {
            case R5618:
                // NOT TUNED on the 2026 robot -- 2025 port values.
                return sharedFollower()
                        .forwardZeroPowerAcceleration(-48.3)
                        .lateralZeroPowerAcceleration(-85.5);
            case R6494:
            default:
                // Zero Power Acceleration Tuners, 6494, 2026-11-05, fresh battery:
                // forward -44.06, -44.06, -47.85; lateral -94.05, -105.5, -104.
                return sharedFollower()
                        .forwardZeroPowerAcceleration(-45.32)
                        .lateralZeroPowerAcceleration(-101.18);
        }
    }

    /** Follower settings that are still the same on both robots. */
    private static FollowerConstants sharedFollower() {
        return new FollowerConstants()
                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0, 0.01))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.00001, 0.6, 0.01))
                .mass(7.7); // kg -- 2025 value, weigh the 2026 robots
    }

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
     * xVelocity/yVelocity are per robot too: measured with Tuning > Automatic >
     * Forward/Lateral Velocity Tuner, fresh battery.
     */
    public static MecanumConstants driveConstants() {
        switch (WhichRobot.get()) {
            case R5618:
                // Per-wheel test on 5618, 2026-11-05: every wheel spun backwards
                // with the old settings, so all four were flipped to these.
                // Velocities NOT TUNED on the 2026 robot -- 2025 port values.
                return sharedDrive()
                        .xVelocity(53.38354732)
                        .yVelocity(40.318533829)
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            case R6494:
            default:
                // Per-wheel test on 6494, 2026-11-05: with 5618's settings the
                // fronts were right and both rears spun backwards, so the rears
                // are flipped relative to 5618.
                // Velocity Tuners, 2026-11-05, fresh battery:
                // forward 69, 69.1, 68.8; lateral 48.95, 49.7, 49.9.
                return sharedDrive()
                        .xVelocity(68.97)
                        .yVelocity(49.52)
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
                .rightFrontMotorName("rf") //EH1
                .rightRearMotorName("rr") //EH0
                .leftRearMotorName("lr") //CH0
                .leftFrontMotorName("lf"); //CH1 - also carries the turret encoder
    }

    /*
     * ODOMETRY (PINPOINT) SETUP: ONE PER ROBOT, like the drive directions.
     * Check with Tuning > Localization > Localization Test (starts at 0,0 facing 0 deg):
     *   push forward 24"  -> x goes UP by 24
     *   push left 24"     -> y goes UP by 24
     *   spin one full turn in place -> total heading +6.28, x/y back within ~1"
     */
    public static PinpointConstants localizerConstants() {
        switch (WhichRobot.get()) {
            case R5618:
                // Localization Test on 5618, 2026-11-05: forward push read y down 24,
                // same as 6494, so the forward pod is REVERSED. Strafe direction and
                // the heading (+6.6 per turn?) still being confirmed.
                return sharedLocalizer()
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            case R6494:
            default:
                // Localization Test on 6494, 2026-11-05: forward push read y 72 -> 48
                // (backwards), so the forward pod is REVERSED. Strafe was right
                // (push right: x 72 -> 96), heading +6.3 per turn, spin drift < 1".
                return sharedLocalizer()
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }
    }

    /** Pinpoint settings that are the same on both robots. */
    private static PinpointConstants sharedLocalizer() {
        return new PinpointConstants()
                // Spin test on 6494 (drift < 1") says these offsets are close enough.
                .forwardPodY(6.89) //175mm 6.89in
                .strafePodX(0) // 0
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint") //CH I2C Bus 0
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants(), hardwareMap)
                .pinpointLocalizer(localizerConstants())
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants())
                .build();
    }
}



