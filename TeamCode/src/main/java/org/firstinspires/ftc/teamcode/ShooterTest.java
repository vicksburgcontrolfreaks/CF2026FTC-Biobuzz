package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooting.ShooterLogic;
import org.firstinspires.ftc.teamcode.subsystems.CollectionSystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Bring-up for the non-blocking Shooter. Answers, in this order:
 *   1. Do both wheels reach the target and agree? (no "WHEELS DISAGREE")
 *   2. Does READY come on within ~1 s, or is it stuck "SLOW SPIN-UP"?
 *   3. Does ONE shot actually launch a piece? Raise FEED_SEC until it does,
 *      and no further. Watch "Last dip": a real shot sags the wheels.
 *   4. Does a 3-piece volley fire 3 pieces, each one going the same distance?
 *      If the 2nd and 3rd fall short, recovery isn't long enough: lower
 *      TOLERANCE_RPM or raise SETTLE_SEC. If 2 pieces leave together, PULSE
 *      feed is pushing too long; if it stalls between pieces, try CONTINUOUS.
 * Tune ShooterLogic and Shooter live in Panels while this runs.
 *
 * Controls (gamepad1):
 *   A           - wheels on / off
 *   D-pad up/dn - target RPM +/- RPM_STEP (applies live)
 *   Right bumper- fire 1        Left bumper - fire 3
 *   B           - stop everything
 *   Y (hold)    - collector in, to load pieces
 *   X (hold)    - indexer unjam (only when the shooter isn't feeding)
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    public static double START_RPM = 1500;
    public static double RPM_STEP = 50;

    private boolean aPrev, upPrev, downPrev, rbPrev, lbPrev, bPrev;

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware();
        hw.init(hardwareMap);
        CollectionSystem collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);
        Shooter shooter = new Shooter(hw.shooterLeft, hw.shooterRight, collection);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.refreshClass(ShooterTest.class);
        PanelsConfigurables.refreshClass(Shooter.class);
        PanelsConfigurables.refreshClass(ShooterLogic.class);

        telemetry.addLine("SHOOTER TEST");
        telemetry.addLine("A = wheels | dpad = RPM | RB = fire 1 | LB = fire 3 | B = stop");
        telemetry.update();

        waitForStart();

        double rpm = START_RPM;
        boolean wheelsOn = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !aPrev) {
                wheelsOn = !wheelsOn;
                if (!wheelsOn) shooter.stop();
            }
            if (gamepad1.dpad_up && !upPrev) rpm += RPM_STEP;
            if (gamepad1.dpad_down && !downPrev) rpm = Math.max(0, rpm - RPM_STEP);
            if (gamepad1.right_bumper && !rbPrev) shooter.fire(1);
            if (gamepad1.left_bumper && !lbPrev) shooter.fire(3);
            if (gamepad1.b && !bPrev) {
                wheelsOn = false;
                shooter.stop();
            }
            aPrev = gamepad1.a;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            bPrev = gamepad1.b;

            if (wheelsOn) shooter.spinUp(rpm);

            if (gamepad1.y) collection.collect(); else collection.stopCollector();
            // The shooter owns the indexer only mid-feed; otherwise it's ours.
            if (!shooter.ownsIndexer()) {
                if (gamepad1.x) collection.unjam(); else collection.stopIndexer();
            }

            shooter.update();

            telemetryM.addData("State", shooter.getState());
            telemetryM.addData("Note", shooter.getNote());
            telemetryM.addData("Ready", shooter.isReady());
            telemetryM.addLine("");
            telemetryM.addData("Dial RPM", rpm);
            telemetryM.addData("Target RPM", shooter.getTargetRpm());
            telemetryM.addData("Left RPM", shooter.getLeftRpm());
            telemetryM.addData("Right RPM", shooter.getRightRpm());
            telemetryM.addLine("");
            telemetryM.addData("Queued / fired", shooter.getShotsQueued() + " / " + shooter.getShotsFired());
            telemetryM.addData("Last dip (RPM)", shooter.getLastDipRpm());
            telemetryM.addData("Indexer amps", collection.getIndexerCurrentAmps());
            telemetryM.update(telemetry);
        }

        shooter.stop();
        shooter.update();
        collection.stopAll();
    }
}
