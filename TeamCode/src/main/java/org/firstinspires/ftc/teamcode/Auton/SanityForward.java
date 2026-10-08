package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Sanity check: drive straight forward for 1 second, slowly.
 *
 * No coordinates, no Pedro Pathing, no odometry -- just all four wheels
 * forward at the same power. This tests ONLY the drive motors.
 *   - Drives straight forward  -> motor directions are correct
 *   - Goes backward/sideways/spins -> a motor direction is wrong
 *
 * Motor names and directions are copied from pedroPathing/Constants.java.
 * If you change them there, change them here too.
 */
@Autonomous(name = "Sanity: Drive Forward", group = "Test")
public class SanityForward extends OpMode {

    private static final double POWER = 0.3;          // slow, for safety
    private static final double DRIVE_SECONDS = 1.0;

    private DcMotor lf, lr, rf, rr;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        timer.reset();
        setAllPower(POWER);
    }

    @Override
    public void loop() {
        if (!done && timer.seconds() > DRIVE_SECONDS) {
            setAllPower(0);
            done = true;
        }
        telemetry.addData("Status", done ? "Done" : "Driving forward");
        telemetry.update();
    }

    @Override
    public void stop() {
        setAllPower(0);
    }

    private void setAllPower(double power) {
        lf.setPower(power);
        lr.setPower(power);
        rf.setPower(power);
        rr.setPower(power);
    }
}
