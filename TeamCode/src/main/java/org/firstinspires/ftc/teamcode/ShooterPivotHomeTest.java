package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Bring-up op mode for the shooter pivot, and the quickest way to check the IR
 * beam break on its own.
 *
 * The config needs "pivotHome" (DigitalChannel). "shooterPivot" (Servo) is
 * OPTIONAL -- tryGet returns null instead of throwing, so this runs on a bench
 * setup where only the sensor is wired. Without the servo the beam readout still
 * works; homing and jogging do not, because both need something to move.
 *
 * WHAT TO DO WITH THIS, IN ORDER:
 *
 * 1. CHECK THE BEAM POLARITY FIRST. Before pressing anything, block the beam
 *    with a finger and watch "Beam Broken". If it says false while blocked,
 *    flip ShooterPivot.BEAM_BROKEN_STATE. Homing cannot work until this is
 *    right, and everything after this step depends on it. This step needs only
 *    the sensor -- you can do it before the pivot servo exists.
 *
 * 2. HOME IT (A). If it fails, the "Home Failure" line says why in words.
 *
 * 3. CHECK REPEATABILITY (A again, a few times). Watch "Zero Drift". Under
 *    about 0.005 is good. Bigger than that and the servo is not finishing its
 *    steps before the beam gets read -- raise FINE_DWELL_MS and try again. Do
 *    not move on until the zero is repeatable, or every offset you measure next
 *    is built on sand.
 *
 * 4. MEASURE THE OFFSETS. Put a pollen ball between the flywheels and jog with
 *    the bumpers until it just fits. Read "Offset From Zero" and write it into
 *    ShooterPivot.POLLEN_OFFSET. Repeat with a nectar ball for NECTAR_OFFSET.
 *
 * 5. VERIFY (B and X). Those jump to the stored offsets. If a ball of each kind
 *    fits at its own button, the pivot is calibrated and done.
 */
@TeleOp(name = "Shooter Pivot Home Test", group = "Test")
public class ShooterPivotHomeTest extends LinearOpMode {

    private static final double JOG_STEP = 0.002;

    @Override
    public void runOpMode() {
        DigitalChannel beam = hardwareMap.get(DigitalChannel.class, "pivotHome");
        beam.setMode(DigitalChannel.Mode.INPUT);

        // Optional, so the sensor can be tested before the servo is wired.
        Servo pivotServo = hardwareMap.tryGet(Servo.class, "shooterPivot");
        ShooterPivot pivot = (pivotServo != null)
                ? new ShooterPivot(pivotServo, beam)
                : null;

        int homeAttempts = 0;
        double lastZero = Double.NaN;
        double zeroDrift = Double.NaN;

        telemetry.addLine("Block the beam with a finger and check 'Beam Broken' BEFORE homing.");
        telemetry.addData("Beam Broken", ShooterPivot.isBeamBroken(beam));
        telemetry.addLine();
        telemetry.addLine(pivot != null
                ? "shooterPivot found. A: home   Bumpers: jog   B: pollen   X: nectar"
                : "shooterPivot NOT configured - sensor readout only, homing unavailable");
        telemetry.update();

        waitForStart();

        boolean aPrev = false, bPrev = false, xPrev = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            if (pivot != null) {
                if (a && !aPrev) {
                    double previousZero = pivot.isHomed() ? pivot.getZeroPosition() : Double.NaN;
                    boolean ok = pivot.home();
                    homeAttempts++;
                    if (ok) {
                        if (!Double.isNaN(previousZero)) {
                            zeroDrift = Math.abs(pivot.getZeroPosition() - previousZero);
                        }
                        lastZero = pivot.getZeroPosition();
                    }
                }
                if (b && !bPrev) {
                    pivot.goToPollen();
                }
                if (x && !xPrev) {
                    pivot.goToNectar();
                }

                // Jog, so the offsets can be measured against a real ball.
                if (gamepad1.right_bumper) {
                    pivot.setPosition(pivot.getCommandedPosition() + JOG_STEP);
                }
                if (gamepad1.left_bumper) {
                    pivot.setPosition(pivot.getCommandedPosition() - JOG_STEP);
                }
            }

            aPrev = a;
            bPrev = b;
            xPrev = x;

            // Always available: the sensor itself.
            telemetry.addData("Beam Broken", ShooterPivot.isBeamBroken(beam));
            telemetry.addData("Raw getState()", beam.getState());
            telemetry.addLine();

            if (pivot == null) {
                telemetry.addLine("shooterPivot NOT configured.");
                telemetry.addLine("Sensor readout works; homing needs the servo.");
                telemetry.update();
                continue;
            }

            telemetry.addLine("A: home   Bumpers: jog   B: pollen   X: nectar");
            telemetry.addLine();
            telemetry.addData("Homed", pivot.isHomed());
            telemetry.addData("Home Attempts", homeAttempts);
            if (pivot.getHomeFailure() != null) {
                telemetry.addData("Home Failure", pivot.getHomeFailure());
            }
            telemetry.addLine();
            telemetry.addData("Zero Position", pivot.isHomed() ? lastZero : Double.NaN);
            telemetry.addData("Zero Drift (want < 0.005)",
                    Double.isNaN(zeroDrift) ? "home twice to measure" : zeroDrift);
            telemetry.addLine();
            telemetry.addData("Commanded Position", pivot.getCommandedPosition());
            telemetry.addData("Offset From Zero <-- WRITE THIS DOWN",
                    pivot.isHomed() ? pivot.getOffsetFromZero() : Double.NaN);
            telemetry.addLine();
            telemetry.addData("Stored POLLEN_OFFSET", ShooterPivot.POLLEN_OFFSET);
            telemetry.addData("Stored NECTAR_OFFSET", ShooterPivot.NECTAR_OFFSET);
            telemetry.update();
        }
    }
}
