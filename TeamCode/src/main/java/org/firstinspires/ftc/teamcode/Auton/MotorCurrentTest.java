package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "MotorCurrentTest")
public class MotorCurrentTest extends LinearOpMode {

    private DcMotorEx lf, rf, lr, rr;

    @Override
    public void runOpMode() {
        // Map motors from hardware config
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        // Set directions for mechanum (adjust if needed)
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to test");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Variables to track current sums and sample count
        double lfCurrentSum = 0, rfCurrentSum = 0, lrCurrentSum = 0, rrCurrentSum = 0;
        int sampleCount = 0;

        // Run motors at 0.5 power
        lf.setPower(0.5);
        rf.setPower(0.5);
        lr.setPower(0.5);
        rr.setPower(0.5);

        while (opModeIsActive() && timer.seconds() < 5) {
            // Get currents in amps
            double lfCurrent = lf.getCurrent(CurrentUnit.AMPS);
            double rfCurrent = rf.getCurrent(CurrentUnit.AMPS);
            double lrCurrent = lr.getCurrent(CurrentUnit.AMPS);
            double rrCurrent = rr.getCurrent(CurrentUnit.AMPS);

            // Accumulate currents
            lfCurrentSum += lfCurrent;
            rfCurrentSum += rfCurrent;
            lrCurrentSum += lrCurrent;
            rrCurrentSum += rrCurrent;
            sampleCount++;

            // Display current readings
            telemetry.addData("Right Rear Current", rrCurrent);
            telemetry.addData("Right Front Current", rfCurrent);
            telemetry.addData("Left Front Current", lfCurrent);
            telemetry.addData("Left Rear Current", lrCurrent);
            telemetry.update();

            sleep(100); // Update every 0.1 sec
        }

        // Stop motors
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);

        // Calculate averages
        double lfAverage = sampleCount > 0 ? lfCurrentSum / sampleCount : 0;
        double rfAverage = sampleCount > 0 ? rfCurrentSum / sampleCount : 0;
        double lrAverage = sampleCount > 0 ? lrCurrentSum / sampleCount : 0;
        double rrAverage = sampleCount > 0 ? rrCurrentSum / sampleCount : 0;

        // Display average currents
        telemetry.addData("Right Rear Avg Current", rrAverage);
        telemetry.addData("Right Front Avg Current", rfAverage);
        telemetry.addData("Left Front Avg Current", lfAverage);
        telemetry.addData("Left Rear Avg Current", lrAverage);
        telemetry.addData("Status", "Test complete");
        telemetry.update();

        // Keep telemetry on screen until opmode stops
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}