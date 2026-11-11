package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    /**
     * The ONE place the collector and indexer directions are set. Every op mode
     * that gets these motors -- through RobotHardware or straight from the
     * hardwareMap -- must call this. Pass null for a motor you don't use.
     *
     * HISTORY: the 2026-10-29 check ran in Collection System Test, which got the
     * motors raw (default FORWARD) -- so FORWARD/FORWARD is what was actually
     * confirmed. But it was recorded as "correct as set in RobotHardware", which
     * said collector REVERSE. Result: the collector ran backwards in everything
     * built on RobotHardware (Shooter Test, DriveTeleOp) and in the autos, until
     * Shooter Test showed it on 6494, 2026-11-05.
     *
     * collector FORWARD = pulls pieces IN. indexer FORWARD = feeds UP to the shooter.
     * Checked on 6494; not yet checked on 5618.
     *
     * 2026-11-11: indexer motor flipped to REVERSE on BOTH robots (it was running
     * backwards). "Forward" in code still means feed UP; only the raw motor
     * direction changed. NOT robot-tested yet.
     */
    public static void applyCollectionDirections(DcMotorSimple collector, DcMotorSimple indexer) {
        if (collector != null) collector.setDirection(DcMotorSimple.Direction.FORWARD);
        if (indexer != null) indexer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    /**
     * @param withVision false = don't open "Webcam 1" here. The default portal
     *                   below has no BIOBUZZ tag library (so no tag poses), and a
     *                   webcam can only be opened once -- an op mode that sets up
     *                   its own camera (DriveTeleOp, via TurretCamera) passes false.
     */
    public void init(HardwareMap hardwareMap, boolean withVision) {
        // Drive motors (mecanum, match Constants naming).
        // Port map confirmed 2026-10-27: the left side lives on the Control Hub
        // now, which is why these comments no longer all say EH.
        rr = hardwareMap.get(DcMotorEx.class, "rr"); // EH0
        rf = hardwareMap.get(DcMotorEx.class, "rf"); // EH1
        lr = hardwareMap.get(DcMotorEx.class, "lr"); // CH0
        lf = hardwareMap.get(DcMotorEx.class, "lf"); // CH1, carries the turret encoder
        // Directions come from Constants, per robot (5618 and 6494 differ).
        // Pedro sets the same values again when createFollower() runs.
        Constants.applyDriveDirections(lf, lr, rf, rr);

        // Collection system
        collector = hardwareMap.get(DcMotorEx.class, "collector"); // CH3
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");     // EH3
        applyCollectionDirections(collector, indexer);

        // OPTIONAL: not fitted to the robots yet (2026-11-05), and nothing that uses
        // RobotHardware reads it. tryGet returns null instead of crashing INIT, so
        // anything that does start using it must check for null first.
        colorSensor = hardwareMap.tryGet(NormalizedColorSensor.class, "colorSensor"); // EH I2C1

        // Shooter: wheels face each other, so one side is physically reversed and
        // both take the same commanded velocity. Note the two motors are on
        // DIFFERENT hubs (EH2 and CH2) -- each hub runs its own velocity PIDF
        // locally so each wheel regulates itself fine, but if the two ever report
        // speeds that disagree, cross-hub command skew is the first suspect.
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");   // EH2
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight"); // CH2
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPivot = hardwareMap.get(Servo.class, "shooterPivot"); // EH servo 0

        // OPTIONAL: ShooterPivot already runs without its beam break (2026-10-31),
        // and as of 2026-11-05 neither robot's config has "pivotHome" at all. The
        // two analog feedback wires replaced it: "pivotPos" (EH analog 3) for the
        // shooter pivot, and "deflectorPos" (CONTROL hub analog 1) for the
        // deflector. Both robots' configs match on these (checked at the robot
        // 2026-11-06; this comment used to say deflectorPos was EH analog 1,
        // which was wrong). "deflectorPos" is read by the Prematch op mode;
        // nothing reads "pivotPos" yet. null when it's not configured.
        pivotHome = hardwareMap.tryGet(DigitalChannel.class, "pivotHome"); // EH digital 0
        if (pivotHome != null) {
            pivotHome.setMode(DigitalChannel.Mode.INPUT);
        }

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

        // AprilTag vision (DECODE-era defaults -- see init(hardwareMap, withVision)).
        if (!withVision) return;
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
}
