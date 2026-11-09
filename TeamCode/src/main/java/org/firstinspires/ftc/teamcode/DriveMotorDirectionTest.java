package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive Motor Direction Test: spins ONE drive wheel at a time, so you can see
 * exactly which motor goes the wrong way instead of guessing from how the whole
 * robot drives.
 *
 * WHY IT READS Constants.java
 * Pedro Pathing sets the drive motor directions from Constants.driveConstants(),
 * and it OVERWRITES whatever RobotHardware set, in DriveTeleOp too. So
 * Constants.java is the only place drive direction really lives, and this test
 * uses exactly those values. If a wheel is wrong here, fix it in Constants.java
 * and nowhere else.
 *
 * The two robots (5618, 6494) have DIFFERENT drive directions. Constants picks
 * the right set from the config name (see WhichRobot). Check the "Robot" line
 * on the screen, and fix only that robot's block in Constants.java.
 *
 * Config needs only the 4 drive motors: "lf", "lr", "rf", "rr".
 *
 * Controls (gamepad1). The buttons are laid out like the wheels:
 *   Left bumper    - left FRONT  wheel (lf)
 *   Left trigger   - left REAR   wheel (lr)
 *   Right bumper   - right FRONT wheel (rf)
 *   Right trigger  - right REAR  wheel (rr)
 *   A (hold)       - all four together: robot should drive straight FORWARD
 * Each wheel runs only while you hold its button.
 *
 * HOW TO USE
 *   1. ROBOT ON BLOCKS, wheels off the ground.
 *   2. Hold each bumper/trigger one at a time. Every wheel should spin FORWARD:
 *      the top of the wheel rolls toward the FRONT of the robot.
 *   3. Write down any wheel that spins backwards. Flip ONLY that motor's
 *      direction in Constants.java (FORWARD <-> REVERSE), redeploy, and recheck.
 *   4. When all four are right, take it off the blocks, give it room, and hold A.
 *      It should drive straight forward, not turn, strafe, or sit there buzzing.
 */
// Shown again 2026-11-09 for tuning on 6494. Was hidden for competition on
// 2026-11-07; put @Disabled back above @TeleOp to hide it from the DS again.
@TeleOp(name = "Drive Motor Direction Test", group = "Test")
public class DriveMotorDirectionTest extends LinearOpMode {

    // Low on purpose. Enough to see the direction, not enough to launch off the blocks.
    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        MecanumConstants drive = Constants.driveConstants();

        DcMotorEx lf = setUp(drive.leftFrontMotorName, drive.leftFrontMotorDirection);
        DcMotorEx lr = setUp(drive.leftRearMotorName, drive.leftRearMotorDirection);
        DcMotorEx rf = setUp(drive.rightFrontMotorName, drive.rightFrontMotorDirection);
        DcMotorEx rr = setUp(drive.rightRearMotorName, drive.rightRearMotorDirection);

        BuildInfo.show(telemetry);
        telemetry.addLine("ROBOT ON BLOCKS. Directions from Constants.java:");
        showDirections(drive);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean all = gamepad1.a;

            boolean runLf = all || gamepad1.left_bumper;
            boolean runLr = all || gamepad1.left_trigger > 0.5;
            boolean runRf = all || gamepad1.right_bumper;
            boolean runRr = all || gamepad1.right_trigger > 0.5;

            lf.setPower(runLf ? TEST_POWER : 0);
            lr.setPower(runLr ? TEST_POWER : 0);
            rf.setPower(runRf ? TEST_POWER : 0);
            rr.setPower(runRr ? TEST_POWER : 0);

            BuildInfo.show(telemetry);
            telemetry.addLine("Every wheel should spin FORWARD (top rolls toward the front).");
            telemetry.addLine();
            telemetry.addData("Running", running(runLf, runLr, runRf, runRr, all));
            telemetry.addLine();
            telemetry.addLine("LB = left front    RB = right front");
            telemetry.addLine("LT = left rear     RT = right rear");
            telemetry.addLine("A  = all four (off the blocks: drives forward)");
            telemetry.addLine();
            showDirections(drive);
            telemetry.update();
        }
    }

    private DcMotorEx setUp(String name, com.qualcomm.robotcore.hardware.DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void showDirections(MecanumConstants drive) {
        telemetry.addData("lf (left front)", drive.leftFrontMotorDirection);
        telemetry.addData("lr (left rear)", drive.leftRearMotorDirection);
        telemetry.addData("rf (right front)", drive.rightFrontMotorDirection);
        telemetry.addData("rr (right rear)", drive.rightRearMotorDirection);
    }

    private String running(boolean lf, boolean lr, boolean rf, boolean rr, boolean all) {
        if (all) return "ALL FOUR";
        StringBuilder s = new StringBuilder();
        if (lf) s.append("left front  ");
        if (lr) s.append("left rear  ");
        if (rf) s.append("right front  ");
        if (rr) s.append("right rear  ");
        return s.length() == 0 ? "nothing" : s.toString();
    }
}
