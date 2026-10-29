package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
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
 *
 * There is no "turretEncoder" device, and there cannot be -- see init().
 */
public class RobotHardware {

    // REV HD Hex bare motor. Both collection motors run a 20:1 gearbox.
    public static final double TICKS_PER_MOTOR_REV = 28;
    public static final double COLLECTOR_GEAR_RATIO = 20.0;
    public static final double INDEXER_GEAR_RATIO = 20.0;
    // The shooter flywheels are direct-driven off the output shaft, so this 3:1
    // is the HD Hex motor's own INTERNAL gearbox, not external gearing
    // (confirmed 2026-10-29). Careful if these are REV UltraPlanetary
    // cartridges -- their printed ratios are nominal, and the "3:1" cartridge is
    // really 2.89:1. Use 2.89 if so.
    public static final double SHOOTER_GEAR_RATIO = 3.0;

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

    // ---- Turret -------------------------------------------------------------
    /** Owns the "turret" CR servo, the "turretHome" switch, and lf's encoder. */
    public Turret turret = new Turret();

    // ---- Vision -------------------------------------------------------------
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

    public void init(HardwareMap hardwareMap) {
        // Drive motors (mecanum, match Constants naming).
        // Port map confirmed 2026-10-27: the left side lives on the Control Hub
        // now, which is why these comments no longer all say EH.
        rr = hardwareMap.get(DcMotorEx.class, "rr"); // EH0
        rf = hardwareMap.get(DcMotorEx.class, "rf"); // EH1
        lr = hardwareMap.get(DcMotorEx.class, "lr"); // CH0
        lf = hardwareMap.get(DcMotorEx.class, "lf"); // CH1, carries the turret encoder
        // Directions from Constants
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Collection system
        collector = hardwareMap.get(DcMotorEx.class, "collector"); // CH3
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");     // EH3
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO(on-robot): confirm which way the indexer has to spin to feed the
        // shooter, and flip this if a ball goes the wrong direction.
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor"); // EH I2C1

        // Shooter: wheels face each other, so one side is physically reversed and
        // both take the same commanded velocity. Note the two motors are on
        // DIFFERENT hubs (EH2 and CH2) -- each hub runs its own velocity PIDF
        // locally so each wheel regulates itself fine, but if the two ever report
        // speeds that disagree, cross-hub command skew is the first suspect.
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");   // EH2
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight"); // CH2
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPivot = hardwareMap.get(Servo.class, "shooterPivot"); // EH servo 0

        pivotHome = hardwareMap.get(DigitalChannel.class, "pivotHome"); // EH digital 0
        pivotHome.setMode(DigitalChannel.Mode.INPUT);

        // Turret. Handed the lf object we already have, so there is one lf in the
        // whole robot rather than two handles fighting over it.
        //
        // WHY THERE IS NO "turretEncoder" IN THE CONFIG: the turret's REV Through
        // Bore encoder is plugged into lf's spare encoder connector, so the SDK
        // reads it through the lf MOTOR. There is no separate device to add, and
        // adding one would not do anything. Two rules keep sharing that port safe,
        // and breaking either one breaks the turret:
        //   1. lf stays in RUN_WITHOUT_ENCODER (the default). Put it in
        //      RUN_USING_ENCODER or RUN_TO_POSITION and the hub's PID would try to
        //      regulate a drive wheel using the TURRET's position.
        //   2. Nobody calls STOP_AND_RESET_ENCODER on lf, which would zero the
        //      turret's position mid-match.
        // Nothing in this repo breaks either rule today, including Pedro's Mecanum.
        //
        // Moved from rf to lf on 2026-10-27: lf is on the Control Hub, so turret
        // encoder reads no longer cross the RS485 link to the Expansion Hub. The
        // angle math is unchanged -- readRawTicks() cancels the motor's direction
        // out either way -- but the cable physically moved, so turret direction
        // and homing both need re-verifying on the robot.
        turret.init(hardwareMap, lf); // "turret" = CH servo 0

        // AprilTag vision
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
}
