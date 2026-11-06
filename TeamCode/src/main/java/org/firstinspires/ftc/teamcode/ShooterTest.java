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
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

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
 *   D-pad left  - pivot to POLLEN gap    D-pad right - pivot to NECTAR gap
 *   Back        - pivot OPEN (pull a stuck ball out)
 *                 Pivot buttons work on 6494 only: its positions were measured
 *                 there. On 5618 they'd hit a hard stop, so they're blocked
 *                 until 5618's band is measured with Pivot Jog Test.
 *                 The pivot does NOT move until you press one.
 *   Y (hold)    - collector in at COLLECT_RPM (200), to load pieces. Works while firing too: hold Y
 *                 during a shot if the collector has to push a ball into the indexer.
 *   RT (hold)   - indexer FEED toward the shooter, 50 RPM (manual; checks direction)
 *   X (hold)    - indexer UNJAM: runs BACKWARD, away from the shooter, 50 RPM
 *                 Both only when the shooter isn't feeding. Speeds: CollectionSystem
 *                 FEED_RPM / UNJAM_RPM. A fired shot feeds at FEED_RPM too.
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    public static double START_RPM = 1500;
    public static double RPM_STEP = 50;

    private boolean aPrev, upPrev, downPrev, rbPrev, lbPrev, bPrev;
    private boolean leftPrev, rightPrev, backPrev;

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware();
        hw.init(hardwareMap);
        CollectionSystem collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);
        Shooter shooter = new Shooter(hw.shooterLeft, hw.shooterRight, collection);

        // Pivot: only on the robot whose positions were measured (6494). Nothing
        // is written to the servo until a pivot button is pressed.
        ShooterPivot pivot = new ShooterPivot(hw.shooterPivot);
        boolean pivotAllowed = WhichRobot.get() == WhichRobot.Robot.R6494;
        String pivotSetting = pivotAllowed ? "not set - press dpad L/R"
                : "BLOCKED on " + WhichRobot.number() + " - measure with Pivot Jog Test first";

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.refreshClass(ShooterTest.class);
        PanelsConfigurables.refreshClass(Shooter.class);
        PanelsConfigurables.refreshClass(ShooterLogic.class);

        telemetry.addLine("SHOOTER TEST");
        BuildInfo.show(telemetry);
        telemetry.addLine("A = wheels | dpad U/D = RPM | RB = fire 1 | LB = fire 3 | B = stop");
        telemetry.addLine("dpad L = pollen gap | dpad R = nectar gap | Back = open");
        telemetry.addLine("Y = collector in | RT = indexer feed | X = indexer unjam (backward)");
        telemetry.addData("Pivot", pivotSetting);
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

            // Pivot presets (6494 only -- see the controls comment).
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right, back = gamepad1.back;
            if ((left && !leftPrev) || (right && !rightPrev) || (back && !backPrev)) {
                if (!pivotAllowed) {
                    // pivotSetting already explains why; leave the servo alone.
                } else if (left && !leftPrev) {
                    pivot.goToPollen();
                    pivotSetting = String.format("POLLEN (%.3f)", pivot.getCommandedPosition());
                } else if (right && !rightPrev) {
                    pivot.goToNectar();
                    pivotSetting = String.format("NECTAR (%.3f)", pivot.getCommandedPosition());
                } else if (pivot.goToOpen()) {
                    pivotSetting = String.format("OPEN (%.3f)", pivot.getCommandedPosition());
                }
            }
            leftPrev = left;
            rightPrev = right;
            backPrev = back;

            if (wheelsOn) shooter.spinUp(rpm);

            if (gamepad1.y) collection.collect(); else collection.stopCollector();
            // The shooter owns the indexer only mid-feed; otherwise it's ours.
            if (!shooter.ownsIndexer()) {
                if (gamepad1.right_trigger > 0.5) collection.feed();
                else if (gamepad1.x) collection.unjam();
                else collection.stopIndexer();
            }

            shooter.update();

            telemetryM.addData("Robot", WhichRobot.number());
            telemetryM.addData("Pivot", pivotSetting);
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
            telemetryM.addData("Collector RPM", collection.getCollectorRPM());
            telemetryM.addData("Indexer RPM", collection.getIndexerRPM());
            telemetryM.addData("Indexer amps", collection.getIndexerCurrentAmps());
            telemetryM.update(telemetry);
        }

        shooter.stop();
        shooter.update();
        collection.stopAll();
    }
}
