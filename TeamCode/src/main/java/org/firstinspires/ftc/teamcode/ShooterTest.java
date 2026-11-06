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
 *                 Fire = feed until a wheel DIPS (the ball went through), stop the
 *                 indexer for PAUSE_SEC and until the wheels recover, then the next.
 *                 No dip in NO_BALL_SEC = "no ball?" and it stops. (ShooterLogic DIP mode)
 *                 The SHOT LOG on screen records each fire (fed / dip / paused time,
 *                 and "EXTRA DIP" if a 2nd ball got through during the pause) so you
 *                 can read what happened afterward. START clears it.
 *   B           - stop everything
 *   D-pad left  - pivot to POLLEN gap    D-pad right - pivot to NECTAR gap
 *   Back        - pivot OPEN (pull a stuck ball out)
 *                 Pivot buttons work on 6494 only: its positions were measured
 *                 there. On 5618 they'd hit a hard stop, so they're blocked
 *                 until 5618's band is measured with Pivot Jog Test.
 *                 The pivot does NOT move until you press one.
 *   Y (hold)    - collector in at COLLECT_RPM (200), to load pieces. Works while firing too: hold Y
 *                 during a shot if the collector has to push a ball into the indexer.
 *   RT (hold)   - indexer FEED toward the shooter at FEED_RPM, NO auto-pause. Hold it until a ball
 *                 shoots, then let go: the screen records that shot's DIP (how far
 *                 each wheel sagged below target). "Wobble, no ball" is the sag
 *                 with nothing feeding. START resets the wobble.
 *   X (hold)    - indexer UNJAM: runs BACKWARD, away from the shooter, 50 RPM
 *                 Both only when the shooter isn't feeding. Speeds: CollectionSystem
 *                 FEED_RPM / UNJAM_RPM. A fired shot feeds at FEED_RPM too.
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    // 1300 chosen on 6494, 2026-11-06, with the indexer at 25 RPM. That's about
    // the wheels' max at full power (HD Hex + 3:1 + flywheel; 1500 never reached
    // READY). With no headroom, a tired battery may never reach READY, and the
    // wheels recover slowly between shots. If READY won't come on, drop to 1250.
    public static double START_RPM = 1300;
    public static double RPM_STEP = 50;

    private boolean aPrev, upPrev, downPrev, rbPrev, lbPrev, bPrev;
    private boolean leftPrev, rightPrev, backPrev, startPrev;

    // ---- Dip measurement (manual feed) ---------------------------------------
    // A ball going through the flywheels makes them sag for a moment. To build
    // "fire = feed until it shoots", we need to know how big a real shot's dip
    // is compared with the wheels' normal wobble with no ball. RT (manual feed)
    // records the biggest drop below target while it's held; the wobble line
    // records the biggest drop while the wheels just spin with nothing feeding.
    private boolean manualFeedPrev;
    private double feedMinL, feedMinR;
    private boolean feedStartedReady;
    private String lastManualDip = "none yet - hold RT until a ball shoots";
    private final java.util.LinkedList<String> recentDips = new java.util.LinkedList<>();
    private double wobble = 0;           // biggest drop below target, no ball
    private double lastTarget = Double.NaN;
    private boolean wheelsOnPrev;
    private final com.qualcomm.robotcore.util.ElapsedTime quiet =
            new com.qualcomm.robotcore.util.ElapsedTime();
    /** Wobble only counts after the wheels have run this long with nothing feeding. */
    private static final double QUIET_SEC = 1.5;

    // ---- Shot log ------------------------------------------------------------
    // Records what fire (RB / LB) actually did, so you can read it AFTER the
    // shots instead of watching a fraction-of-a-second pause live. START clears it.
    //   fed     = how long the indexer ran before the dip (ball reached the wheels)
    //   dip     = how far the wheels sagged for that shot
    //   paused  = how long the indexer stayed stopped before the next feed / READY
    //   "!! EXTRA DIP" = the wheels dipped again while the indexer was STOPPED,
    //   i.e. a second ball got through on its own -- the pause can't stop that.
    private final java.util.LinkedList<String> shotLog = new java.util.LinkedList<>();
    private final com.qualcomm.robotcore.util.ElapsedTime logClock =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private ShooterLogic.State prevState = ShooterLogic.State.IDLE;
    private int prevFired = 0;
    private boolean prevNoBall = false;
    private double feedStartSec, pauseStartSec, shotFedSec;
    private int pendingShot = 0;          // shot # waiting for its pause/dip to finish
    private boolean recoveredSinceShot;   // wheels came back near target after the dip
    private boolean extraDipLogged;

    private void logLine(String line) {
        shotLog.addFirst(line);
        while (shotLog.size() > 6) shotLog.removeLast();
    }

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

            // ---- Dip measurement ----
            double target = shooter.getTargetRpm();
            double l = shooter.getLeftRpm(), r = shooter.getRightRpm();
            boolean manualFeed = !shooter.ownsIndexer() && gamepad1.right_trigger > 0.5;

            if (manualFeed && !manualFeedPrev) {          // RT just pressed
                feedMinL = l;
                feedMinR = r;
                feedStartedReady = shooter.isReady();
            }
            if (manualFeed) {
                feedMinL = Math.min(feedMinL, l);
                feedMinR = Math.min(feedMinR, r);
            }
            if (!manualFeed && manualFeedPrev && target > 0) {  // RT just released
                lastManualDip = String.format("L %.0f / R %.0f%s",
                        target - feedMinL, target - feedMinR,
                        feedStartedReady ? "" : "  (wheels weren't READY - ignore)");
                recentDips.addFirst(String.format("%.0f/%.0f", target - feedMinL, target - feedMinR));
                while (recentDips.size() > 5) recentDips.removeLast();
            }
            manualFeedPrev = manualFeed;

            // Wobble: only while the wheels have been spinning a while with nothing
            // feeding. Reset by START, a new target, or turning the wheels on.
            if (manualFeed || shooter.ownsIndexer() || target != lastTarget
                    || (wheelsOn && !wheelsOnPrev) || !wheelsOn) {
                quiet.reset();
            }
            if ((wheelsOn && !wheelsOnPrev) || target != lastTarget) wobble = 0;
            if (gamepad1.start && !startPrev) {
                wobble = 0;
                shotLog.clear();
            }

            // ---- Shot log ----
            double nowSec = logClock.seconds();
            ShooterLogic.State st = shooter.getState();
            int fired = shooter.getShotsFired();
            double drop = target - Math.min(l, r);
            if (st == ShooterLogic.State.FEEDING && prevState != ShooterLogic.State.FEEDING) {
                feedStartSec = nowSec;
            }
            if (fired > prevFired) {                       // fire logic saw a dip
                shotFedSec = nowSec - feedStartSec;
                pauseStartSec = nowSec;
                pendingShot = fired;
                recoveredSinceShot = false;
                extraDipLogged = false;
            }
            if (st == ShooterLogic.State.RECOVERING) {
                if (drop < ShooterLogic.TOLERANCE_RPM) recoveredSinceShot = true;
                if (recoveredSinceShot && !extraDipLogged && drop >= ShooterLogic.DIP_RPM) {
                    logLine(String.format("!! EXTRA DIP %.0f during pause - a 2nd ball got through", drop));
                    extraDipLogged = true;
                }
            }
            if (pendingShot > 0 && prevState == ShooterLogic.State.RECOVERING
                    && st != ShooterLogic.State.RECOVERING) {   // pause over
                logLine(String.format("#%d fed %.1fs -> dip %.0f -> paused %.1fs",
                        pendingShot, shotFedSec, shooter.getLastDipRpm(), nowSec - pauseStartSec));
                pendingShot = 0;
            }
            // A big dip knocks the logic out of READY in the same loop, so look at
            // the loop where it LEFT READY without a feed starting.
            if (prevState == ShooterLogic.State.READY && st != ShooterLogic.State.FEEDING && pendingShot == 0
                    && !manualFeed && drop >= ShooterLogic.DIP_RPM && !extraDipLogged) {
                logLine(String.format("!! DIP %.0f while READY, no feed - a ball got through?", drop));
                extraDipLogged = true;
            }
            if (st == ShooterLogic.State.READY && drop < ShooterLogic.TOLERANCE_RPM) extraDipLogged = false;
            if (shooter.wasNoBall() && !prevNoBall) {
                logLine(String.format("no ball? fed %.1fs with no dip", ShooterLogic.NO_BALL_SEC));
            }
            prevNoBall = shooter.wasNoBall();
            prevFired = fired;
            prevState = st;
            if (wheelsOn && target > 0 && quiet.seconds() > QUIET_SEC) {
                wobble = Math.max(wobble, Math.max(target - l, target - r));
            }
            lastTarget = target;
            wheelsOnPrev = wheelsOn;
            startPrev = gamepad1.start;

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
            telemetryM.addLine("");
            telemetryM.addLine("-- DIP MEASUREMENT (RT = feed until it shoots) --");
            if (manualFeed) {
                telemetryM.addData("Feeding... dip so far", String.format("L %.0f / R %.0f",
                        target - feedMinL, target - feedMinR));
            }
            telemetryM.addData("Last shot dip", lastManualDip);
            telemetryM.addData("Recent (L/R, newest first)", recentDips.isEmpty() ? "-" : String.join("  ", recentDips));
            telemetryM.addLine("");
            telemetryM.addLine("-- SHOT LOG (RB/LB fire, newest first; START clears) --");
            if (shotLog.isEmpty()) telemetryM.addLine("  (no shots yet)");
            for (String line : shotLog) telemetryM.addLine("  " + line);
            telemetryM.addData("Wobble, no ball", quiet.seconds() > QUIET_SEC || wobble > 0
                    ? String.format("%.0f  (START resets)", wobble) : "settling...");
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
