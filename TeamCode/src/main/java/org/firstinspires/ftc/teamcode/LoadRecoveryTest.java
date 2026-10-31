package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LoadRecovery;

/**
 * Head-to-head test of the two ways to rescue a ball lost in the dead space
 * between the collector and the indexer. Both maneuvers live in LoadRecovery;
 * this drives them and measures what actually happened.
 *
 * Put a ball in the dead spot by hand, then try each button against it. Same
 * ball, same spot, so the comparison means something.
 *
 * WHAT IT NEEDS IN THE CONFIG
 *   "lf" "lr" "rf" "rr"  DcMotorEx, required -- the chassis shake drives them raw
 *   "collector"          DcMotorEx, required
 *   "indexer"            DcMotorEx, required
 *   "pinpoint"           required, via Constants.createFollower()
 * No vision, no turret, no shooter: this does not build RobotHardware, so it
 * runs with a partly configured robot and does not pay for camera startup.
 *
 * CONTROLS (gamepad1)
 *   A   chassis shake    snap forward hard, coast, creep back. SHAKE_REPS times
 *   B   collector pulse  reverse the collector briefly, then drive it in
 *   X   cancel whatever is running, right now
 *   Y   zero the drift measurement at the current pose
 *
 * READ THE DIRECTION WARNING BEFORE PRESSING A
 * The shake assumes all four wheels at positive power drives the robot FORWARD,
 * so a loose ball shifts rearward into the indexer. If it is backward, the ball
 * gets thrown toward the front of the robot and may leave it. This is not
 * hypothetical: RobotHardware.init() sets all four drive directions exactly
 * inverted from pedroPathing/Constants.java, and the Constants values -- the
 * ones that win -- have only ever been verified on the 2025 robot.
 *
 * SO, FIRST RUN, IN THIS ORDER:
 *   1. No ball in the robot. Set SNAP_POWER to about 0.3 from the dashboard
 *   2. Press A and watch which way the robot lurches FIRST
 *   3. Forward is right. Backward means flip LoadRecovery.SNAP_SIGN to -1.0
 *   4. Put SNAP_POWER back to 1.0 and carry on
 *
 * WHAT TO WATCH, ONCE IT IS POINTED THE RIGHT WAY
 *
 * 1. "Snap displacement" -- how far the robot actually got during the burst. If
 *    this is near zero the robot never accelerated: pinned on a wall, wheels
 *    slipping, or SNAP_MS too short to matter. A dead reading here is a blocked
 *    robot, not a failed idea, and the difference matters.
 *
 * 2. "Net drift" -- where the robot ended up versus where it started, over the
 *    whole sequence. Tune LoadRecovery.RETURN_MS against this: longer if the
 *    robot creeps forward, shorter if it walks backward. It will never be zero,
 *    because the snap slips the wheels. Small enough to ignore is the goal.
 *
 * 3. "Indexer peak amps" -- the one honest signal that a ball actually arrived.
 *    A nectar ball loads the indexer about 2 A above its 0.79 A idle, which is
 *    unmistakable. A POLLEN ball costs the indexer essentially nothing, so this
 *    stays flat whether the rescue worked or not. Watch the ball, not the number,
 *    when testing with pollen -- see BallLoadClassifier on why there is no
 *    ball-presence signal anywhere on this robot.
 *
 * 4. Which maneuver actually frees the ball, and after how many reps. If the
 *    shake needs all four, raise SHAKE_REPS. If rep one does it every time, drop
 *    it -- this is time spent not scoring.
 *
 * If NEITHER works, that is a real result and worth writing down: it points at
 * the ball being wedged rather than resting, which is geometry and needs the
 * linkage or the ramp changed, not code.
 */
@Configurable
@TeleOp(name = "Load Recovery Test", group = "Test")
public class LoadRecoveryTest extends LinearOpMode {

    private double maxSnapDisplacement = 0.0;
    private double peakIndexerAmps = 0.0;
    private String lastRun = "nothing run yet";

    private boolean aPrev, bPrev, xPrev, yPrev;

    @Override
    public void runOpMode() {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        DcMotorEx collector = hardwareMap.get(DcMotorEx.class, "collector");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        // Built for two reasons: it is what sets the authoritative drive
        // directions (see the class comment), and it gives Pinpoint odometry so
        // the shake can be measured instead of guessed at.
        Follower follower = Constants.createFollower(hardwareMap);

        LoadRecovery recovery =
                new LoadRecovery(lf, lr, rf, rr, collector, indexer);

        telemetry.addLine("LOAD RECOVERY TEST");
        telemetry.addLine("A: chassis shake   B: collector pulse");
        telemetry.addLine("X: cancel   Y: zero the drift reading");
        telemetry.addLine();
        telemetry.addLine("FIRST RUN: no ball, SNAP_POWER ~0.3, press A and");
        telemetry.addLine("check the robot lurches FORWARD. If not, set");
        telemetry.addLine("LoadRecovery.SNAP_SIGN = -1.0 before going further.");
        telemetry.update();

        waitForStart();

        Pose reference = follower.getPose();
        Pose runStart = reference;

        while (opModeIsActive()) {
            // Localization first. LoadRecovery.update() writes motor powers and
            // has to run AFTER this -- Pedro's drivetrain can write zeros while
            // no path is being followed, and whoever writes last wins.
            follower.update();
            Pose now = follower.getPose();

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (a && !aPrev && !recovery.isRunning()) {
                runStart = now;
                maxSnapDisplacement = 0.0;
                peakIndexerAmps = 0.0;
                recovery.startShake();
                lastRun = "chassis shake";
            }
            if (b && !bPrev && !recovery.isRunning()) {
                runStart = now;
                maxSnapDisplacement = 0.0;
                peakIndexerAmps = 0.0;
                recovery.startCollectorPulse();
                lastRun = "collector pulse";
            }
            if (x && !xPrev && recovery.isRunning()) {
                recovery.cancel();
                lastRun += " (cancelled)";
            }
            if (y && !yPrev) {
                reference = now;
                runStart = now;
                maxSnapDisplacement = 0.0;
            }
            aPrev = a;
            bPrev = b;
            xPrev = x;
            yPrev = y;

            recovery.update();

            if (recovery.isRunning()) {
                maxSnapDisplacement =
                        Math.max(maxSnapDisplacement, distance(runStart, now));
                peakIndexerAmps = Math.max(peakIndexerAmps,
                        indexer.getCurrent(CurrentUnit.AMPS));
            }

            telemetry.addLine("A: shake   B: pulse   X: cancel   Y: zero drift");
            telemetry.addLine();
            telemetry.addData("State", recovery.getStatus());
            telemetry.addData("Last run", lastRun);

            telemetry.addLine();
            telemetry.addData("Snap displacement (in, peak)", maxSnapDisplacement);
            if (recovery.isShaking() && maxSnapDisplacement < 0.25) {
                telemetry.addLine("  near zero - robot may be pinned or slipping");
            }
            telemetry.addData("Net drift from reference (in)", distance(reference, now));
            telemetry.addData("Indexer peak amps", peakIndexerAmps);
            telemetry.addLine("  nectar shows ~2A over 0.79A idle; pollen shows nothing");

            telemetry.addLine();
            telemetry.addData("Pose", String.format("x %.1f  y %.1f  h %.1f deg",
                    now.getX(), now.getY(), Math.toDegrees(now.getHeading())));
            telemetry.update();
        }

        recovery.cancel();
    }

    private static double distance(Pose from, Pose to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        return Math.hypot(dx, dy);
    }
}
