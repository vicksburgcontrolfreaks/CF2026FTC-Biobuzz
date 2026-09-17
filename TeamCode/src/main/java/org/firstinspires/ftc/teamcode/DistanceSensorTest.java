package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Test")
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        waitForStart();
        while (opModeIsActive()) {
            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("Object Detected", RobotHardware.isObjectDetected(distanceInches));
            telemetry.update();
        }
    }
}
