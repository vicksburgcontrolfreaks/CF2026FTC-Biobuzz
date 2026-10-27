package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Every physical device on the robot, grabbed once and handed out to whoever needs it.
 *
 * DEPRECATED AND REMOVED 2026-10-27 -- do not add these back:
 *   "sensor1" "sensor2" "sensor3"  three magazine beam breaks (and isMagazineFull())
 *   "flipper"                      launch servo, replaced by the indexer motor
 *   "shooter"                      single shooter motor, now shooterLeft + shooterRight
 *   "limitSwitch"                  unused touch sensor (only ever on Elliott's branches)
 *
 * NAMING NOTE: "indexer" used to be a Servo on Jade's branches -- the one that sets the
 * gap between the two flywheels. That servo is now "shooterPivot", and "indexer" is the
 * collection-system MOTOR. If an op mode asks for a Servo named "indexer" it is stale.
 */
public class RobotHardware {

    // REV HD Hex bare motor. Both collection motors run a 20:1 gearbox.
    public static final double TICKS_PER_MOTOR_REV = 28;
    public static final double COLLECTOR_GEAR_RATIO = 20.0;
    public static final double INDEXER_GEAR_RATIO = 20.0;

    // ---- Drive (names match Pedro Pathing's Constants) ----------------------
    public DcMotorEx rf, rr, lr, lf;

    // ---- Collection system --------------------------------------------------
    public DcMotorEx collector;
    public DcMotorEx indexer;
    /** Reads game-piece color at the indexer wheel. Wrap with Jade's ColorSensing. */
    public NormalizedColorSensor colorSensor;

    // ---- Shooter ------------------------------------------------------------
    public DcMotorEx shooterLeft, shooterRight;
    /** Sets the gap between the two flywheels (pollen vs. nectar spacing). */
    public Servo shooterPivot;
    /** IR beam break the pivot homes against. See ShooterPivot for why. */
    public DigitalChannel pivotHome;

    // ---- Vision -------------------------------------------------------------
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

    public void init(HardwareMap hardwareMap) {
        // Drive motors (mecanum, match Constants naming)
        rf = hardwareMap.get(DcMotorEx.class, "rf"); // EH1
        rr = hardwareMap.get(DcMotorEx.class, "rr"); // EH0
        lr = hardwareMap.get(DcMotorEx.class, "lr"); // EH3
        lf = hardwareMap.get(DcMotorEx.class, "lf"); // EH2
        // Directions from Constants
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Collection system
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO(on-robot): confirm which way the indexer has to spin to feed the
        // shooter, and flip this if a ball goes the wrong direction.
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Shooter: wheels face each other, so one side is physically reversed and
        // both take the same commanded velocity.
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPivot = hardwareMap.get(Servo.class, "shooterPivot");

        pivotHome = hardwareMap.get(DigitalChannel.class, "pivotHome");
        pivotHome.setMode(DigitalChannel.Mode.INPUT);

        // AprilTag vision
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
}
