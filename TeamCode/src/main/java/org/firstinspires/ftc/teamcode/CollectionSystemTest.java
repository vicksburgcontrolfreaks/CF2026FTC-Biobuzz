package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.CollectionSystem;

/**
 * Bring-up op mode for the collection system. Standalone -- the active config
 * only needs "collector", "indexer" and "colorSensor", so this runs before the
 * drivetrain or camera are configured.
 *
 * WHAT TO WATCH FOR, GIVEN WHAT IS NEW HERE:
 *
 * 1. DIRECTIONS, BEFORE ANYTHING ELSE. Hold A and confirm the collector pulls a
 *    piece IN rather than spitting it out. Hold right bumper and confirm the
 *    indexer feeds UP toward the shooter. If either runs backwards, fix the
 *    setDirection call in RobotHardware -- not by negating powers here, or the
 *    two files will disagree forever.
 *
 * 2. THE 20:1 RATIO. "Collector RPM" should read roughly 300 at full power, not
 *    6000. If it reads about 5x high, the gear ratio constant in RobotHardware
 *    is wrong -- that exact bug was in TestTeleOp until 2026-10-27.
 *
 * 3. JAM CURRENT. Let the indexer stall against a deliberately stuck piece for
 *    a second and note "Indexer Amps". That number is the threshold any future
 *    auto-unjam logic needs, and guessing it is how you get a robot that
 *    unjams constantly during a match.
 */
@TeleOp(name = "Collection System Test", group = "Test")
public class CollectionSystemTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx collector = hardwareMap.get(DcMotorEx.class, "collector");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        NormalizedColorSensor colorSensor =
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        CollectionSystem collection = new CollectionSystem(collector, indexer, colorSensor);
        // A braked indexer would crush a piece caught mid-feed.
        collection.setFloatOnZeroPower();

        telemetry.addLine("A: collect   B: eject   RB: feed   LB: unjam");
        telemetry.addLine("Check directions FIRST -- see the notes on this op mode.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                collection.collect();
            } else if (gamepad1.b) {
                collection.eject();
            } else {
                collection.stopCollector();
            }

            if (gamepad1.right_bumper) {
                collection.feed();
            } else if (gamepad1.left_bumper) {
                collection.unjam();
            } else {
                collection.stopIndexer();
            }

            telemetry.addLine("A: collect   B: eject   RB: feed   LB: unjam");
            telemetry.addLine();
            telemetry.addData("Collector RPM (expect ~300 max)", collection.getCollectorRPM());
            telemetry.addData("Indexer RPM", collection.getIndexerRPM());
            telemetry.addData("Indexer Amps", collection.getIndexerCurrentAmps());
            telemetry.addLine();
            // Raw channels only -- piece classification belongs to Jade's
            // ColorSensing, not to this bring-up op mode.
            telemetry.addData("Color (raw argb)", colorSensor.getNormalizedColors().toColor());
            telemetry.update();
        }

        collection.stopAll();
    }
}
