package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Touch Sensor Test", group = "Test")
public class TouchSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Pressed", limitSwitch.isPressed());
            telemetry.update();
        }
    }
}
