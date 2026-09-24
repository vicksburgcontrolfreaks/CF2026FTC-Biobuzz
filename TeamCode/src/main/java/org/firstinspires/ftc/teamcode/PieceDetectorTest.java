package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

// Standalone test - only needs "colorSensor" and "indexer" in the active config,
// not the full RobotHardware map. Holds a game piece in front of the sensor and
// watches the indexer servo react: yellow -> position 0.0, red/blue -> position 1.0.
@TeleOp(name = "Piece Detector Test", group = "Test")
public class PieceDetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        Servo indexer = hardwareMap.get(Servo.class, "indexer");

        ColorSensing colorSensing = new ColorSensing(colorSensor);
        PieceDetector pieceDetector = new PieceDetector(colorSensing, indexer);

        waitForStart();
        while (opModeIsActive()) {
            pieceDetector.update();

            telemetry.addData("Piece", colorSensing.classify());
            telemetry.addData("Hue", colorSensing.getHue());
            telemetry.addData("Distance (cm)", colorSensing.getDistanceCM());
            telemetry.addData("Reliable?", colorSensing.isReadingReliable());
            telemetry.addData("Indexer Position", indexer.getPosition());
            telemetry.update();
        }
    }
}
