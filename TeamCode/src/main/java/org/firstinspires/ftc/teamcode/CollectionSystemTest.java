package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.CollectionSystem;

/**
 * Bring-up op mode for the collection system. Standalone -- the active config
 * only needs "collector" and "indexer", so this runs before the drivetrain or
 * camera are configured. "colorSensor" is OPTIONAL -- it is not fitted to the
 * robot yet, so tryGet returns null instead of throwing and the colour readout
 * is simply skipped. Nothing about the collector or indexer depends on it.
 *
 * CLOSED LOOP SINCE 2026-10-29. Both motors now take a target RPM through
 * setVelocity() instead of a raw power level, so a number on the screen is a
 * real speed the hub chases rather than a guess at a throttle position. The two
 * targets start at the speeds that work on the real collector:
 *   collector 200 RPM, indexer 50 RPM
 *
 * CONTROLS (gamepad1)
 *   A                 collector in      B   collector out
 *   RB                indexer feed      LB  indexer unjam (reverse)
 *   D-pad up/down     collector target +/- 50 RPM
 *   D-pad left/right  indexer target +/- 10 RPM
 *
 * Both motors are momentary -- they run while the button is held and stop when
 * it is released. The D-pad only changes the TARGET, so it is safe to dial a
 * speed in before touching a game piece.
 *
 * WHAT TO WATCH FOR, GIVEN WHAT IS NEW HERE:
 *
 * 1. DIRECTIONS, BEFORE ANYTHING ELSE. Hold A and confirm the collector pulls a
 *    piece IN rather than spitting it out. Hold right bumper and confirm the
 *    indexer feeds UP toward the shooter. If either runs backwards, fix the
 *    setDirection call in RobotHardware -- not by negating targets here, or the
 *    two files will disagree forever.
 *
 * 2. TARGET VERSUS ACTUAL. This is what closed loop buys you. Roughly 300 RPM is
 *    all these motors have through the 20:1, so a 200 target should hold steady
 *    and a 300 one will sag below target because the motor is saturated. Sagging
 *    at a high target is not bad tuning -- it is asking for more than the motor
 *    has. Lower the target.
 *
 *    If actual reads about 5x the target instead, the gear ratio constant in
 *    RobotHardware is wrong -- that exact bug was in TestTeleOp until 2026-10-27.
 *
 * 3. JAM CURRENT. Let the indexer stall against a deliberately stuck piece for
 *    a second and note "Indexer Amps". That number is the threshold any future
 *    auto-unjam logic needs, and guessing it is how you get a robot that
 *    unjams constantly during a match.
 */
@TeleOp(name = "Collection System Test", group = "Test")
public class CollectionSystemTest extends LinearOpMode {

    // Starting targets: the speeds that work on the real collector.
    private static final double COLLECTOR_START_RPM = 200.0;
    private static final double INDEXER_START_RPM = 50.0;

    // D-pad step sizes. The collector moves in bigger jumps because it runs an
    // order of magnitude faster than the indexer does.
    private static final double COLLECTOR_STEP_RPM = 50.0;
    private static final double INDEXER_STEP_RPM = 10.0;

    // Both motors are the same 20:1 gearbox, so they share a ceiling. Past about
    // 300 the motor is saturated and actual RPM stops following the target.
    private static final double MAX_RPM = 300.0;

    private double collectorTargetRpm = COLLECTOR_START_RPM;
    private double indexerTargetRpm = INDEXER_START_RPM;

    private boolean upPrev, downPrev, leftPrev, rightPrev;

    @Override
    public void runOpMode() {
        DcMotorEx collector = hardwareMap.get(DcMotorEx.class, "collector");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        NormalizedColorSensor colorSensor =
                hardwareMap.tryGet(NormalizedColorSensor.class, "colorSensor");

        CollectionSystem collection = new CollectionSystem(collector, indexer, colorSensor);
        // A braked indexer would crush a piece caught mid-feed.
        collection.setFloatOnZeroPower();

        telemetry.addLine("A: collect   B: eject   RB: feed   LB: unjam");
        telemetry.addLine("D-pad U/D: collector +/-50 RPM   L/R: indexer +/-10 RPM");
        telemetry.addLine("Check directions FIRST -- see the notes on this op mode.");
        telemetry.addData("Color sensor", colorSensor != null ? "found" : "not fitted (fine)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            adjustTargets();

            if (gamepad1.a) {
                collection.setCollectorRPM(collectorTargetRpm);
            } else if (gamepad1.b) {
                // Eject is the same speed, backwards.
                collection.setCollectorRPM(-collectorTargetRpm);
            } else {
                collection.stopCollector();
            }

            if (gamepad1.right_bumper) {
                collection.setIndexerRPM(indexerTargetRpm);
            } else if (gamepad1.left_bumper) {
                collection.setIndexerRPM(-indexerTargetRpm);
            } else {
                collection.stopIndexer();
            }

            telemetry.addLine("A: collect   B: eject   RB: feed   LB: unjam");
            telemetry.addLine("D-pad U/D: collector +/-50 RPM   L/R: indexer +/-10 RPM");
            telemetry.addLine();
            telemetry.addData("Collector target RPM", collectorTargetRpm);
            telemetry.addData("Collector actual RPM", collection.getCollectorRPM());
            telemetry.addLine();
            telemetry.addData("Indexer target RPM", indexerTargetRpm);
            telemetry.addData("Indexer actual RPM", collection.getIndexerRPM());
            telemetry.addData("Indexer Amps", collection.getIndexerCurrentAmps());
            telemetry.addLine();
            // Raw channels only -- piece classification belongs to Jade's
            // ColorSensing, not to this bring-up op mode.
            if (colorSensor != null) {
                telemetry.addData("Color (raw argb)",
                        colorSensor.getNormalizedColors().toColor());
            } else {
                telemetry.addLine("Color sensor not fitted - colour readout skipped");
            }
            telemetry.update();
        }

        collection.stopAll();
    }

    /**
     * One step per press, not one per loop. Without the rising-edge check a
     * single tap would run through the whole range in a fraction of a second,
     * because this loop runs dozens of times while your thumb is still down.
     */
    private void adjustTargets() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !upPrev) {
            collectorTargetRpm += COLLECTOR_STEP_RPM;
        }
        if (down && !downPrev) {
            collectorTargetRpm -= COLLECTOR_STEP_RPM;
        }
        if (right && !rightPrev) {
            indexerTargetRpm += INDEXER_STEP_RPM;
        }
        if (left && !leftPrev) {
            indexerTargetRpm -= INDEXER_STEP_RPM;
        }

        collectorTargetRpm = Range.clip(collectorTargetRpm, 0.0, MAX_RPM);
        indexerTargetRpm = Range.clip(indexerTargetRpm, 0.0, MAX_RPM);

        upPrev = up;
        downPrev = down;
        leftPrev = left;
        rightPrev = right;
    }
}
