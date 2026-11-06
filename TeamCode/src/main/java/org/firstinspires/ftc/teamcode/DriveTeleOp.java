package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutoMenu;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.geometry.FieldCentric;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.NectarGuard;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.teamcode.vision.TurretCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/*
 * Match TeleOp. Gamepad 1 DRIVES, gamepad 2 runs everything else.
 *
 * GAMEPAD 1 (driver)
 *   Left stick   drive (field-centric: forward = away from OUR drivers)
 *   Right stick  turn
 *   BACK         "the robot faces away from me right now" -- heading reset, rumbles
 *
 * GAMEPAD 2 (operator)                             (all "hold" = only while held)
 *   RB (hold)    FIRE: one ball at a time, only when the flywheel is at speed,
 *                pausing for it to recover after each shot
 *   LB (hold)    indexer backward
 *   A  (hold)    collector in            X (hold)   collector out
 *   RT (hold)    collector + indexer forward (with the flywheel on, the indexer
 *                part FIRES like RB)
 *   LT (hold)    collector + indexer backward
 *   Y            flywheel on / off (after a JAM fault: clears it and spins up)
 *   Right stick X  turret by hand -- overrides aiming while held
 *   D-pad Up     park the turret forward / resume aiming
 *   D-pad Down   "a NECTAR is next": nectar gap until the next shot
 *   D-pad L / R  aim at our AUDIENCE / SCORING cell
 *   Rumble       short = ready (at speed + on target); 3 blips = nectar detected;
 *                long = JAM fault, everything stopped
 *
 * TURRET: aims at our cell by camera whenever it's in view (TurretAim). Not
 * homed at Start (no auto since power-up)? It homes itself -- so the turret must
 * start roughly forward, same rule as Turret Test.
 *
 * NECTAR SAFETY: Launcher runs every ball through NectarGuard -- see that class.
 * Its threshold is NOT yet recalibrated for FEED_RPM 25 (see NectarGuard).
 *
 * START POSE (INIT menu, gamepad 1, same controls as Select Auto):
 *   FROM AUTO    - auto saves its pose every loop (PoseHandoff); default if it
 *                  ran in the last AUTO_POSE_FRESH_SEC.
 *   FACING AWAY  - point the robot away from our drivers before Start.
 *                  Position unknown (field center) -- fine for driving.
 */
@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_SPEED = 1.0; // 1.0 = full power. Lower (e.g. 0.5) only for new-driver practice.
    private static final double AUTO_POSE_FRESH_SEC = 60;  // a match has ~8 s between auto and TeleOp
    private static final double TRIGGER_HELD = 0.5;
    private static final double STICK_DEADZONE = 0.1;
    private static final double TURRET_NUDGE_DEG_PER_SEC = 90;  // full stick
    private static final double TURRET_JOG_POWER = 0.25;        // full stick, turret NOT homed (no soft limits!)

    private enum StartPose {
        FROM_AUTO("From auto"), FACING_AWAY("Facing away from us");
        private final String label;
        StartPose(String label) { this.label = label; }
        @Override public String toString() { return label; }
    }

    private enum AimCell {
        SCORING("Scoring cell"), AUDIENCE("Audience cell");
        private final String label;
        AimCell(String label) { this.label = label; }
        @Override public String toString() { return label; }
    }

    private final AutoMenu menu = new AutoMenu();
    private final AutoMenu.Choice<Alliance> alliance =
            menu.add("Alliance", Alliance.RED, Alliance.BLUE);
    private final AutoMenu.Choice<StartPose> startPose =
            menu.add("Start pose", StartPose.FACING_AWAY, StartPose.FROM_AUTO)
                    .allowWhen(p -> p != StartPose.FROM_AUTO || PoseHandoff.has());
    private final AutoMenu.Choice<AimCell> aimCell =
            menu.add("Aim at", AimCell.SCORING, AimCell.AUDIENCE);

    private RobotHardware hardware;
    private Follower follower;
    private Launcher launcher;
    private Turret turret;
    private TurretCamera camera;
    private TurretAim aim;

    private final Launcher.Controls controls = new Launcher.Controls();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean turretParked = false;
    private boolean turretManual = false;
    private boolean wasReady = false;
    private int lastTrips = 0;
    private boolean wasFault = false;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, false);   // TurretCamera owns the webcam
        follower = Constants.createFollower(hardwareMap);
        launcher = new Launcher(hardware, hardwareMap);
        turret = hardware.turret;
        camera = new TurretCamera(hardwareMap);

        if (PoseHandoff.has() && PoseHandoff.ageSeconds() < AUTO_POSE_FRESH_SEC) {
            if (PoseHandoff.alliance() != null) alliance.set(PoseHandoff.alliance());
            startPose.set(StartPose.FROM_AUTO);
        }
    }

    @Override
    public void init_loop() {
        menu.update(gamepad1);
        camera.update();

        BuildInfo.show(telemetry);
        menu.show(telemetry);
        telemetry.addLine("");
        if (PoseHandoff.has()) {
            Pose p = PoseHandoff.pose();
            telemetry.addData("Auto pose", "(%.0f, %.0f) %.0f deg, %s, %.0f s ago%s",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()),
                    PoseHandoff.alliance(), PoseHandoff.ageSeconds(),
                    PoseHandoff.ageSeconds() < AUTO_POSE_FRESH_SEC ? "" : "  (OLD)");
            if (startPose.get() == StartPose.FROM_AUTO
                    && PoseHandoff.alliance() != null && PoseHandoff.alliance() != alliance.get()) {
                telemetry.addLine(">>> Auto ran as " + PoseHandoff.alliance()
                        + " but Alliance is " + alliance.get() + " -- check!");
            }
        } else {
            telemetry.addData("Auto pose", "none (no auto since power-up)");
        }
        if (startPose.get() == StartPose.FACING_AWAY) {
            telemetry.addLine("Point the robot AWAY from the " + alliance.get() + " drivers before Start.");
        }
        telemetry.addData("Turret", "%s (%s)", turret.getState(), turret.getStatusNote());
        if (!turret.isHomed() && Turret.isMeasured()) {
            telemetry.addLine("Turret will HOME at Start -- make sure it's roughly forward.");
        }
        if (launcher.isPivotBlocked()) {
            telemetry.addLine(">>> Pivot NOT measured on robot " + WhichRobot.number()
                    + ": pivot won't move, nectar guard can't protect.");
        }
        telemetry.addData("Camera", camera.getNote());
        telemetry.update();
    }

    @Override
    public void start() {
        Pose start;
        if (startPose.get() == StartPose.FROM_AUTO && PoseHandoff.has()) {
            start = PoseHandoff.pose();
        } else {
            start = new Pose(FieldConstants.CENTER, FieldConstants.CENTER,
                    alliance.get().awayFromDriversHeading());
        }
        follower.setStartingPose(start);
        PoseHandoff.clear();   // taken; a second TeleOp run must not reuse it

        aim = new TurretAim(cellFor(aimCell.get()).tagIds);   // default: search for OUR cell only
        if (!turret.isHomed() && Turret.isMeasured()) turret.startHoming();
        launcher.start();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        follower.update();   // localization only -- we drive the motors ourselves
        camera.update();

        drive();
        operateLauncher();
        operateTurret(dt);
        feedback();
        showTelemetry(dt);
    }

    // ---- Gamepad 1 ------------------------------------------------------------

    private void drive() {
        if (gamepad1.backWasPressed()) {
            Pose p = follower.getPose();
            follower.setPose(new Pose(p.getX(), p.getY(), alliance.get().awayFromDriversHeading()));
            gamepad1.rumble(300);
        }

        double[] robot = FieldCentric.toRobot(
                -gamepad1.left_stick_y, gamepad1.left_stick_x,
                follower.getPose().getHeading(), alliance.get().awayFromDriversHeading());
        double forward = robot[0];
        double right = robot[1];
        double turn = gamepad1.right_stick_x;

        double den = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);
        hardware.lf.setPower((forward + right + turn) / den * DRIVE_SPEED);
        hardware.rf.setPower((forward - right - turn) / den * DRIVE_SPEED);
        hardware.lr.setPower((forward - right + turn) / den * DRIVE_SPEED);
        hardware.rr.setPower((forward + right - turn) / den * DRIVE_SPEED);
    }

    // ---- Gamepad 2: balls -----------------------------------------------------

    private void operateLauncher() {
        if (gamepad2.yWasPressed()) launcher.toggleFlywheel();
        if (gamepad2.dpadDownWasPressed()) launcher.requestNectar();

        controls.fire = gamepad2.right_bumper;
        controls.indexerBack = gamepad2.left_bumper;
        controls.collectIn = gamepad2.a;
        controls.collectOut = gamepad2.x;
        controls.allForward = gamepad2.right_trigger > TRIGGER_HELD;
        controls.allBack = gamepad2.left_trigger > TRIGGER_HELD;
        launcher.update(controls);
    }

    // ---- Gamepad 2: turret ----------------------------------------------------

    private void operateTurret(double dt) {
        if (gamepad2.dpadLeftWasPressed()) setCell(AimCell.AUDIENCE);
        if (gamepad2.dpadRightWasPressed()) setCell(AimCell.SCORING);

        if (gamepad2.dpadUpWasPressed()) {
            turretParked = !turretParked;
            aim.resetPhase();
            if (turretParked && turret.isHomed()) turret.setTargetAngle(Turret.PARK_ANGLE);
        }

        double stick = gamepad2.right_stick_x;
        boolean manual = Math.abs(stick) > STICK_DEADZONE;
        if (manual) {
            // Stick right = turret turns right (clockwise) = angle goes DOWN.
            if (turret.isHomed()) {
                turret.setTargetAngle(turret.getTargetAngle() - stick * TURRET_NUDGE_DEG_PER_SEC * dt);
            } else if (turret.getState() != Turret.State.HOMING) {
                turret.setRawPower(-stick * TURRET_JOG_POWER);
            }
        } else if (turretManual) {
            if (!turret.isHomed()) turret.setRawPower(0);
            aim.resetPhase();   // let go: start aiming fresh from here
        }
        turretManual = manual;

        List<AprilTagDetection> detections = camera.getDetections();
        boolean aimAllowed = !turretParked && !manual && turret.isHomed();
        aim.update(detections, turret, aimAllowed);
        turret.update();
    }

    private void setCell(AimCell cell) {
        aimCell.set(cell);
        aim.setTargetTagIds(cellFor(cell).tagIds);
        aim.resetPhase();
    }

    private FieldAprilTags.Cell cellFor(AimCell cell) {
        if (alliance.get() == Alliance.RED) {
            return cell == AimCell.SCORING ? FieldAprilTags.Cell.RED_SCORING : FieldAprilTags.Cell.RED_AUDIENCE;
        }
        return cell == AimCell.SCORING ? FieldAprilTags.Cell.BLUE_SCORING : FieldAprilTags.Cell.BLUE_AUDIENCE;
    }

    // ---- Rumble: the operator watches the field, not the Driver Station -------

    private void feedback() {
        NectarGuard guard = launcher.getGuard();

        boolean fault = guard.isFault();
        if (fault && !wasFault) gamepad2.rumble(1000);
        wasFault = fault;

        if (guard.getTrips() > lastTrips) gamepad2.rumbleBlips(3);
        lastTrips = guard.getTrips();

        boolean ready = launcher.isReady() && aim.isOnTarget();
        if (ready && !wasReady) gamepad2.rumble(150);
        wasReady = ready;
    }

    private void showTelemetry(double dt) {
        NectarGuard guard = launcher.getGuard();
        if (guard.isFault()) telemetry.addLine(">>> " + guard.getNote());

        telemetry.addData("Flywheel", "%s  %s  L %.0f / R %.0f (target %.0f)",
                launcher.isFlywheelOn() ? "ON" : "off", launcher.getShooter().getNote(),
                launcher.getShooter().getLeftRpm(), launcher.getShooter().getRightRpm(),
                launcher.getShooter().getTargetRpm());
        telemetry.addData("Gap", "%s  | guard %s: %s", launcher.isPivotBlocked() ? "PIVOT BLOCKED" : launcher.getGap(),
                guard.getState(), guard.getNote());
        telemetry.addData("Turret", "%s %+.0f deg%s | %s", turret.getState(), turret.getAngle(),
                turretParked ? " PARKED" : turretManual ? " MANUAL" : "", aim.getStatus());
        telemetry.addData("Aim at", cellFor(aimCell.get()).displayName);
        telemetry.addLine("");
        Pose p = follower.getPose();
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg  [%s]", p.getX(), p.getY(),
                Math.toDegrees(p.getHeading()), alliance.get());
        // For recalibrating the nectar trip at the real feed speed.
        telemetry.addData("Indexer amps", "now %.2f  last run peak %.2f  (trip %.2f)",
                launcher.getIndexerAmps(), guard.getLastRunPeakAmps(), NectarGuard.TRIP_AMPS);
        // Slow loops = late dip detection and a late nectar trip. Watch this.
        telemetry.addData("Loop", "%.0f ms", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
        if (camera != null) camera.close();
    }
}
