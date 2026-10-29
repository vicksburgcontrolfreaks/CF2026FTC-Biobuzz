package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Simple Motor TeleOp", group = "Prototypes")
public class SimpleMotorTeleOp extends OpMode {

    private DcMotorEx collector;
    private DcMotorEx indexer;
    private DcMotorEx shooterLeft, shooterRight;


    @Override
    public void init() {
        // Grab the motors from hardware config. Names updated 2026-10-27:
        // the single "shooter" became shooterLeft/shooterRight, and "indexer"
        // is now a real DcMotorEx instead of a commented-out maybe.
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        collector = hardwareMap.get(DcMotorEx.class, "collector");

        telemetry.addData("Status", "Motor ready! Plug in and test.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Indexer momentary gamepad 1 A (feed) and Y (unjam)
        if (gamepad1.a) {
            indexer.setPower(1);
        } else if (gamepad1.y) {
            indexer.setPower(-1);
        } else {
            indexer.setPower(0);
        }

        // Shooter momentary control gamepad 1 Left Stick Y.
        // Both wheels take the same power; shooterRight is reversed in the
        // hardware config, not here, so this stays a single value.
        double power = -gamepad1.left_stick_y;  // Flip sign so up = forward
        shooterLeft.setPower(power);
        shooterRight.setPower(power);

        // Collector: D-pad up = pull in, D-pad down = push out, right bumper
        // = half speed. The collector is a MOTOR, not the CR servo it used to be.
        //
        // The final else matters. Without it the collector latched on at whatever
        // it was last set to and kept running after you let go of the button.
        if (gamepad1.dpad_up) {
            collector.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            collector.setPower(-1.0);
        } else if (gamepad1.right_bumper) {
            collector.setPower(0.5);
        } else {
            collector.setPower(0.0);
        }

        // Telemetry for debugging
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Collector Power", collector.getPower());
        telemetry.addData("Indexer Power", indexer.getPower());
        telemetry.addData("Indexer Current (Amps)", indexer.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Safety: everything off when the op mode ends. This used to be a
        // commented-out line referring to a variable that does not exist.
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        collector.setPower(0);
        indexer.setPower(0);
    }
}