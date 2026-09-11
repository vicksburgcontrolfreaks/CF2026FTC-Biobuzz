package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Simple Motor TeleOp", group = "Prototypes")
public class SimpleMotorTeleOp extends OpMode {

    private DcMotorEx collector;  // Our test motor
    private DcMotorEx shooter;  // Our test motor
//    private CRServo   ;


    @Override
    public void init() {
        // Grab the motor and servo from hardware config
//        indexer = hardwareMap.get(DcMotorEx.class, "indexer"); //Indexer CH 0
        shooter = hardwareMap.get(DcMotorEx.class, "shooter"); //Shooter CH 1
        collector = hardwareMap.get(DcMotorEx.class, "collector"); //Shooter CH 1
//        collector = hardwareMap.get(CRServo.class, "collector"); //collector servo CH 0

        // Optional: Reverse direction if motor spins backward
       //  motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Motor ready! Plug in and test.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Indexer momentary gamepad 1 A and Y
//        if(gamepad1.a){
//            indexer.setPower(1);
//        }
//        else if (gamepad1.y){
//            indexer.setPower(-1);
//        }
//        else{
//            indexer.setPower(0);
//        }

        // Shooter momentary control gamepad 1 Left Stick Y
        double power = -gamepad1.left_stick_y;  // Flip sign so up = forward
        shooter.setPower(power);

        // Collector continuous rotation servo control
        if (gamepad1.dpad_up) {
            collector.setPower(1.0); // Rotate forward (adjust value as needed)
        } else if (gamepad1.dpad_down) {
            collector.setPower(0.0); // Rotate reverse (adjust value as needed)
        } else if (gamepad1.right_bumper) {
            collector.setPower(0.5); // Stop the servo
        }

        // Telemetry for debugging
        telemetry.addData("Motor Power", power);
//        telemetry.addData("Collector Servo Position", collector.setPower());
        // telemetry.addData("Current Draw (Amps)", indexer.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Safety: Stop motor on OpMode end
//        motor.setPower(0);
    }
}