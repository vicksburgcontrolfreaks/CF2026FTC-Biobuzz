package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotHardware {
    // Object detection band for distanceSensor (game piece must read within this range)
    private static final double DETECT_MIN_INCHES = 1.0;
    private static final double DETECT_MAX_INCHES = 5.0;

    // Drive motors (names from Constants)
    public DcMotorEx rf, rr, lr, lf;
    // Mechanisms
    public DcMotorEx collector;
    public DcMotorEx shooter;
    public Servo flipper;
    // Sensors (beam breaks)
    public DigitalChannel sensor1, sensor2, sensor3;
    // Sensors (I2C)
    public Rev2mDistanceSensor distanceSensor;
    // Vision
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

        // Mechanisms
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flipper = hardwareMap.get(Servo.class, "flipper");

        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensors
        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensor3 = hardwareMap.get(DigitalChannel.class, "sensor3");
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);
        sensor3.setMode(DigitalChannel.Mode.INPUT);

        // Distance sensor (name must match Driver Station hardware config)
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        // AprilTag vision
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }

    public boolean isMagazineFull() {
        // True if all 3 sensors detect ball (low = true for beam breaks)
        return !sensor1.getState() && !sensor2.getState() && !sensor3.getState();
    }

    public boolean isObjectDetected() {
        return isObjectDetected(distanceSensor.getDistance(DistanceUnit.INCH));
    }

    public static boolean isObjectDetected(double distanceInches) {
        // True if a game piece is within the detection band.
        // Out-of-range readings come back as NaN, which fails both comparisons below.
        return distanceInches >= DETECT_MIN_INCHES && distanceInches <= DETECT_MAX_INCHES;
    }
}