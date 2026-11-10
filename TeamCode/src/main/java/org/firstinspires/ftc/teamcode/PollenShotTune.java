package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutoMenu;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.teamcode.geometry.FieldCentric;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.ShooterLogic;
import org.firstinspires.ftc.teamcode.subsystems.CollectionSystem;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.teamcode.vision.TurretCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayDeque;
import java.util.List;

/*
 * Pollen Shot Tune: find the best collector / indexer / flywheel / pivot /
 * deflector combination for POLLEN, one knob at a time, standing still.
 *
 * STANDALONE ON PURPOSE: no ShooterLogic state machine, no NectarGuard, no
 * at-speed gate, no queueing, no pauses. Hold fire and the indexer runs at the
 * tuned speed, period. That way every knob changes exactly one thing. The
 * shot log MEASURES each shot (flywheel dip, recovery, time between shots) but
 * never acts on it.
 *
 *   >>> POLLEN ONLY. The pivot stays wherever you set it. A nectar fed into a
 *   >>> pollen gap has nothing protecting it here.
 *
 * Still protected (parts, not shooting behavior): the pivot is clipped to its
 * measured safe band (ShooterPivot), and the deflector keeps its soft limits.
 *
 * THE DEFLECTOR STOWS ITSELF IN INIT (2026-11-10). Press INIT and it homes if
 * needed -- down to the hard stop, then up to Deflector.STOW_ANGLE_DEG, the
 * 18 in. pre-match park -- so the stow can be measured without pressing Start.
 * Already zeroed, it just goes there rather than re-homing, because re-homing
 * every INIT means pushing into the stop all session. LB and RB jump between
 * the stow and the shot.
 *
 * That INIT movement is fine in a bench tuner and WRONG in a match op mode:
 * DriveTeleOp homes at Start precisely because G403 forbids moving during the
 * auto-to-TeleOp INIT. Do not copy it.
 *
 * The turret homes itself at Start if needed (start it roughly
 * forward), same as DriveTeleOp.
 *
 * GAMEPAD 1 (tuner + driving)
 *   Sticks         drive, field-centric, same as DriveTeleOp
 *   X              heading reset ("robot faces away from me")
 *   D-pad Up/Down  pick the knob to tune (">" marks it)
 *   D-pad R / L    step it up / down
 *   A (hold)       fine steps while held
 *   Y              reset the picked knob to its starting value
 *   B              clear the shot log
 *   LB             deflector to the STOW angle (18 in. pre-match park)
 *   RB             deflector to the SHOOTING angle
 *
 * GAMEPAD 2 (operator) -- same buttons as DriveTeleOp, but direct, no gating:
 *   B  (hold)      flywheel on + indexer forward
 *   RT (hold)      flywheel on + indexer + collector forward
 *   LT (hold)      indexer + collector backward
 *   A / X (hold)   collector in / out
 *   Y              flywheel on / off
 *   Right stick X  turret by hand (overrides aiming while held)
 *   Left stick     flick up / down: deflector to the next preset (quick jump)
 *   D-pad Up       park the turret / resume aiming
 *   D-pad L / R    aim at our AUDIENCE / SCORING cell
 *   Rumble         short = flywheel at speed
 *
 * The "RECORD THIS" line at the bottom of the screen is everything you need to
 * write down when a combination works.
 */
@Configurable
@TeleOp(name = "Pollen Shot Tune", group = "Tune")
public class PollenShotTune extends OpMode {

    // ---- Knobs -------------------------------------------------------------

    /**
     * Optional pivot calibration, so the screen can show an estimated flywheel
     * gap in inches. Measure the gap (calipers or a drill bit as a gauge) at the
     * robot's stock POLLEN_POSITION and again 0.05 tighter, then fill these in.
     * NaN = not measured, the screen just shows servo position.
     */
    public static double PIVOT_GAP_AT_STOCK_IN = Double.NaN;
    /** Gap change in inches per +1.0 servo position (it gets SMALLER as position goes up). */
    public static double PIVOT_IN_PER_UNIT = Double.NaN;
    public static final double POLLEN_DIAMETER_IN = 2.8;

    private enum Knob {
        //        label        unit    step  fine   min  max   format
        COLLECTOR("Collector", "RPM",  25,   5,     0,   300,  "%.0f"),
        INDEXER  ("Indexer",   "RPM",  10,   2,     10,  150,  "%.0f"),
        FLYWHEEL ("Flywheel",  "RPM",  50,   10,    500, 2500, "%.0f"),
        PIVOT    ("Pivot",     "pos",  0.01, 0.005, 0,   1,    "%.3f"),   // limits = ShooterPivot safe band
        DEFLECTOR("Deflector", "deg",  1,    0.5,   0,   90,   "%.1f");

        final String label, unit, format;
        final double step, fine, min, max;
        Knob(String label, String unit, double step, double fine, double min, double max, String format) {
            this.label = label; this.unit = unit; this.step = step; this.fine = fine;
            this.min = min; this.max = max; this.format = format;
        }
    }

    private static final double START_COLLECTOR_RPM = 200;
    private static final double START_FLYWHEEL_RPM = 1300;
    // The goldilocks shooting angle found on 6494 (Jade, 2026-11-10), measured
    // against the hard-stop zero. Starts the deflector knob at the known-good
    // shot instead of 66, which was from the old level-relative reference.
    //
    // KEEP THIS EQUAL TO RouteAuto.AUTO_ARC_DEG. Same physical angle, written in
    // two places: here is where it gets FOUND, there is where auto USES it. Not
    // referenced directly so a tuner cannot be broken by an op mode it has no
    // business depending on -- but they drift if only one gets changed.
    private static final double START_DEFLECTOR_DEG = 80;

    private final double[] value = new double[Knob.values().length];
    private final double[] start = new double[Knob.values().length];
    private int selected = Knob.FLYWHEEL.ordinal();

    // ---- Driving / turret, same as DriveTeleOp -------------------------------
    private static final double TRIGGER_HELD = 0.5;
    private static final double STICK_DEADZONE = 0.1;
    private static final double TURRET_NUDGE_DEG_PER_SEC = 90;
    private static final double TURRET_JOG_POWER = 0.25;

    private final AutoMenu menu = new AutoMenu();
    private final AutoMenu.Choice<Alliance> alliance = menu.add("Alliance", Alliance.RED, Alliance.BLUE);

    private RobotHardware hw;
    private Follower follower;
    private CollectionSystem collection;
    private ShooterPivot pivot;
    private Deflector deflector;
    private Turret turret;
    private TurretCamera camera;
    private TurretAim aim;
    private FieldAprilTags.Cell targetCell;
    private boolean turretParked = false;
    private boolean turretManual = false;
    private boolean deflectorStickArmed = true;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    // ---- Launcher state (direct, no state machine) ---------------------------
    private boolean flywheelOn = false;
    private double leftRpm, rightRpm;
    private boolean wasAtSpeed = false;

    // ---- Shot log: measurement only ------------------------------------------
    private static final int LOG_SIZE = 5;
    private static class Shot {
        final int n; final double dipRpm, recoverSec, gapSec; final boolean firstOfBurst;
        Shot(int n, double dipRpm, double recoverSec, double gapSec, boolean firstOfBurst) {
            this.n = n; this.dipRpm = dipRpm; this.recoverSec = recoverSec;
            this.gapSec = gapSec; this.firstOfBurst = firstOfBurst;
        }
    }
    private final ArrayDeque<Shot> shots = new ArrayDeque<>();
    private int shotCount = 0;
    private boolean dipArmed = false;      // wheels back in tolerance since the last shot / target change
    private boolean inShot = false;
    private double shotStartSec, shotPeakDrop;
    private double lastShotSec = Double.NaN;
    private double feedStartSec = Double.NaN;   // indexer went forward (start of a burst)
    private boolean firstShotOfBurst = false;
    private double lastTargetRpm = 0;

    @Override
    public void init() {
        hw = new RobotHardware();
        hw.init(hardwareMap, false);   // TurretCamera owns the webcam
        follower = Constants.createFollower(hardwareMap);
        collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);   // loads per-robot FEED_RPM
        pivot = new ShooterPivot(hw.shooterPivot);                                    // loads per-robot positions
        deflector = new Deflector(hardwareMap);
        turret = hw.turret;
        camera = new TurretCamera(hardwareMap);

        for (DcMotorEx m : new DcMotorEx[]{hw.shooterLeft, hw.shooterRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast, don't brake a flywheel
        }

        start[Knob.COLLECTOR.ordinal()] = START_COLLECTOR_RPM;
        start[Knob.INDEXER.ordinal()] = CollectionSystem.FEED_RPM;
        start[Knob.FLYWHEEL.ordinal()] = START_FLYWHEEL_RPM;
        start[Knob.PIVOT.ordinal()] = ShooterPivot.POLLEN_POSITION;
        start[Knob.DEFLECTOR.ordinal()] = START_DEFLECTOR_DEG;
        System.arraycopy(start, 0, value, 0, value.length);

        // Stow the deflector during INIT, so the 18 in. pre-match position can be
        // checked and measured without pressing Start.
        //
        // Not zeroed yet -> home, which already parks at STOW_ANGLE_DEG. Already
        // zeroed -> just go there, because re-homing every INIT means pushing into
        // the hard stop and ~10 s of travel on every run of a tuning session.
        //
        // MOVING IN INIT IS FINE *HERE* AND NOWHERE NEAR A MATCH. DriveTeleOp
        // deliberately homes at Start instead, because during the auto-to-TeleOp
        // transition G403 says INIT must not move anything. This is a bench tuner
        // in the Tune group that never runs in a match -- do not copy the pattern
        // into a match op mode.
        if (deflector.isPresent()) {
            if (!deflector.isZeroed()) {
                deflector.startHoming();
            } else {
                deflector.goTo(Deflector.STOW_ANGLE_DEG);
            }
        }
    }

    @Override
    public void init_loop() {
        menu.update(gamepad1);
        camera.update();
        deflector.update(true);    // allowed to move in INIT: stowing to 45 (see init())

        telemetry.addLine("POLLEN SHOT TUNE -- pollen only, NO nectar guard");
        menu.show(telemetry);
        telemetry.addData("Robot", robotName());
        telemetry.addData("Camera", camera.getNote());
        if (!deflector.isPresent()) {
            telemetry.addLine(">>> No deflector found -- arc can't be tuned.");
        } else if (deflector.isHoming()) {
            telemetry.addData("Deflector", "HOMING -- %s", deflector.getHomeNote());
            telemetry.addLine("Finding the hard stop, then stowing. Give it a few seconds.");
        } else if (!deflector.isZeroed()) {
            telemetry.addLine(">>> Deflector NOT zeroed and homing did not take.");
            telemetry.addLine("    Run Deflector Zero, or it won't move.");
        } else {
            telemetry.addData("Deflector", "now %.1f deg, stowing to %.0f -> %.0f at Start",
                    deflector.getAngle(), Deflector.STOW_ANGLE_DEG, START_DEFLECTOR_DEG);
            telemetry.addLine("LB = stow, RB = shot. Measure the 18 in. at the stow angle.");
        }
        if (!turret.isHomed() && Turret.isMeasured()) {
            telemetry.addLine("Turret will HOME at Start -- make sure it's roughly forward.");
        }
        telemetry.addLine("Point the robot AWAY from the " + alliance.get() + " drivers before Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(FieldConstants.CENTER, FieldConstants.CENTER,
                alliance.get().awayFromDriversHeading()));
        targetCell = cellFor(true);
        aim = new TurretAim(targetCell.tagIds);
        if (!turret.isHomed() && Turret.isMeasured()) turret.startHoming();
        pivot.setPosition(value[Knob.PIVOT.ordinal()]);           // servos move at Start, not INIT
        if (deflector.isZeroed()) deflector.goTo(value[Knob.DEFLECTOR.ordinal()]);
        loopTimer.reset();
        clock.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        follower.update();   // localization only, for field-centric driving
        camera.update();

        drive();
        tune();
        operate();
        operateTurret(dt);
        operateDeflectorFlick();
        logShots();
        showTelemetry(dt);
    }

    // ---- Gamepad 1: driving + tuning -----------------------------------------

    private void drive() {
        if (gamepad1.xWasPressed()) {
            Pose p = follower.getPose();
            follower.setPose(new Pose(p.getX(), p.getY(), alliance.get().awayFromDriversHeading()));
            gamepad1.rumble(300);
        }
        double[] robot = FieldCentric.toRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                follower.getPose().getHeading(), alliance.get().awayFromDriversHeading());
        double forward = robot[0], right = robot[1], turn = gamepad1.right_stick_x;
        double den = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);
        hw.lf.setPower((forward + right + turn) / den);
        hw.rf.setPower((forward - right - turn) / den);
        hw.lr.setPower((forward - right + turn) / den);
        hw.rr.setPower((forward + right - turn) / den);
    }

    private void tune() {
        int n = Knob.values().length;
        if (gamepad1.dpadUpWasPressed()) selected = (selected + n - 1) % n;
        if (gamepad1.dpadDownWasPressed()) selected = (selected + 1) % n;
        if (gamepad1.bWasPressed()) clearLog();

        // Jump the deflector between the only two angles that matter, so the
        // 18 in. stow can be checked without stepping 35 degrees by hand and
        // without leaving the tuner to INIT an auto. Selects the deflector knob
        // too, so the screen shows what just moved.
        if (gamepad1.leftBumperWasPressed()) {
            selected = Knob.DEFLECTOR.ordinal();
            set(Knob.DEFLECTOR, Deflector.STOW_ANGLE_DEG);
        }
        if (gamepad1.rightBumperWasPressed()) {
            selected = Knob.DEFLECTOR.ordinal();
            set(Knob.DEFLECTOR, START_DEFLECTOR_DEG);
        }

        Knob k = Knob.values()[selected];
        double step = gamepad1.a ? k.fine : k.step;
        double v = value[selected];
        if (gamepad1.dpadRightWasPressed()) v += step;
        if (gamepad1.dpadLeftWasPressed()) v -= step;
        if (gamepad1.yWasPressed()) v = start[selected];
        if (v != value[selected]) set(k, v);
    }

    /** Clip, round, store, and apply right away. */
    private void set(Knob k, double v) {
        double min = k == Knob.PIVOT ? ShooterPivot.MIN_SAFE_POSITION : k.min;
        double max = k == Knob.PIVOT ? ShooterPivot.MAX_SAFE_POSITION : k.max;
        v = Math.max(min, Math.min(max, v));
        v = Math.round(v * 1000.0) / 1000.0;   // no 0.5780000001 on the screen
        value[k.ordinal()] = v;
        switch (k) {
            case PIVOT:     pivot.setPosition(v); break;
            case DEFLECTOR: if (deflector.isZeroed()) deflector.goTo(v); break;
            default: break;   // collector / indexer / flywheel are written every loop in operate()
        }
    }

    // ---- Gamepad 2: the launcher, direct --------------------------------------

    private void operate() {
        boolean fire = gamepad2.b;
        boolean allForward = gamepad2.right_trigger > TRIGGER_HELD;
        boolean allBack = gamepad2.left_trigger > TRIGGER_HELD;

        if (gamepad2.yWasPressed()) flywheelOn = !flywheelOn;
        if (fire || allForward) flywheelOn = true;

        // Flywheels
        double target = flywheelOn ? value[Knob.FLYWHEEL.ordinal()] : 0;
        if (target != lastTargetRpm) {   // a new target isn't a shot: wait for the wheels to settle again
            dipArmed = false;
            inShot = false;
            lastTargetRpm = target;
        }
        if (target <= 0) {
            hw.shooterLeft.setPower(0);   // coast down
            hw.shooterRight.setPower(0);
        } else {
            if (!Double.isNaN(Shooter.P) && !Double.isNaN(Shooter.I)
                    && !Double.isNaN(Shooter.D) && !Double.isNaN(Shooter.F)) {
                hw.shooterLeft.setVelocityPIDFCoefficients(Shooter.P, Shooter.I, Shooter.D, Shooter.F);
                hw.shooterRight.setVelocityPIDFCoefficients(Shooter.P, Shooter.I, Shooter.D, Shooter.F);
            }
            hw.shooterLeft.setVelocity(rpmToTicksPerSec(target));
            hw.shooterRight.setVelocity(rpmToTicksPerSec(target));
        }
        leftRpm = ticksPerSecToRpm(hw.shooterLeft.getVelocity());
        rightRpm = ticksPerSecToRpm(hw.shooterRight.getVelocity());

        // Indexer: straight through, no gate
        boolean indexerForward = fire || allForward;
        if (indexerForward) {
            collection.setIndexerRPM(value[Knob.INDEXER.ordinal()]);
            if (Double.isNaN(feedStartSec)) {
                feedStartSec = clock.seconds();
                firstShotOfBurst = true;
            }
        } else {
            feedStartSec = Double.NaN;
            if (allBack) collection.unjam();
            else collection.stopIndexer();
        }

        // Collector
        double coll = value[Knob.COLLECTOR.ordinal()];
        if (allForward || (gamepad2.a && !allBack)) collection.setCollectorRPM(coll);
        else if (allBack || gamepad2.x) collection.setCollectorRPM(-coll);
        else collection.stopCollector();

        // Rumble when the wheels reach speed
        boolean atSpeed = isAtSpeed();
        if (atSpeed && !wasAtSpeed) gamepad2.rumble(150);
        wasAtSpeed = atSpeed;
    }

    private boolean isAtSpeed() {
        double t = flywheelOn ? value[Knob.FLYWHEEL.ordinal()] : 0;
        return t > 0 && Math.abs(leftRpm - t) <= ShooterLogic.TOLERANCE_RPM
                && Math.abs(rightRpm - t) <= ShooterLogic.TOLERANCE_RPM;
    }

    // ---- Gamepad 2: turret and deflector flick ---------------------------------

    private void operateTurret(double dt) {
        List<AprilTagDetection> detections = camera.getDetections();
        // Our cells, camera picks (like DriveTeleOp's default); D-pad L/R overrides.
        setTarget(HiveGeometry.cameraCell(alliance.get(), hiveIdsIn(detections), targetCell));
        if (gamepad2.dpadLeftWasPressed()) setTarget(cellFor(false));
        if (gamepad2.dpadRightWasPressed()) setTarget(cellFor(true));
        if (gamepad2.dpadUpWasPressed()) {
            turretParked = !turretParked;
            aim.resetPhase();
            if (turretParked && turret.isHomed()) turret.setTargetAngle(Turret.PARK_ANGLE);
        }

        double stick = gamepad2.right_stick_x;
        boolean manual = Math.abs(stick) > STICK_DEADZONE;
        if (manual) {
            if (turret.isHomed()) {
                turret.setTargetAngle(turret.getTargetAngle() - stick * TURRET_NUDGE_DEG_PER_SEC * dt);
            } else if (turret.getState() != Turret.State.HOMING) {
                turret.setRawPower(-stick * TURRET_JOG_POWER);
            }
        } else if (turretManual) {
            if (!turret.isHomed()) turret.setRawPower(0);
            aim.resetPhase();
        }
        turretManual = manual;

        aim.update(detections, turret, !turretParked && !manual && turret.isHomed());
        turret.update();
    }

    private void operateDeflectorFlick() {
        double y = -gamepad2.left_stick_y;
        if (deflectorStickArmed && Math.abs(y) > 0.6 && deflector.isZeroed()) {
            if (y > 0) deflector.stepUp(); else deflector.stepDown();
            value[Knob.DEFLECTOR.ordinal()] = deflector.getTarget();   // keep the knob in sync
            deflectorStickArmed = false;
        } else if (Math.abs(y) < 0.3) {
            deflectorStickArmed = true;
        }
        deflector.update(true);
    }

    // ---- Shot log ---------------------------------------------------------------
    // A shot = the slower wheel sags DIP_RPM below target after having been back
    // within TOLERANCE_RPM. Recovery = from the sag until it's back within
    // tolerance. Gap = time since the previous shot (or, for the first ball of a
    // press, since the indexer started: that's the "fed" time).

    private void logShots() {
        double target = lastTargetRpm;
        if (target <= 0) return;
        double now = clock.seconds();
        double drop = target - Math.min(leftRpm, rightRpm);

        if (!inShot) {
            if (drop < ShooterLogic.TOLERANCE_RPM) dipArmed = true;
            if (dipArmed && drop >= ShooterLogic.DIP_RPM) {
                inShot = true;
                dipArmed = false;
                shotStartSec = now;
                shotPeakDrop = drop;
            }
            return;
        }
        shotPeakDrop = Math.max(shotPeakDrop, drop);
        if (drop < ShooterLogic.TOLERANCE_RPM) {
            boolean first = firstShotOfBurst && !Double.isNaN(feedStartSec);
            double gap = first ? shotStartSec - feedStartSec
                    : Double.isNaN(lastShotSec) ? Double.NaN : shotStartSec - lastShotSec;
            shots.addFirst(new Shot(++shotCount, shotPeakDrop, now - shotStartSec, gap, first));
            while (shots.size() > LOG_SIZE) shots.removeLast();
            firstShotOfBurst = false;
            lastShotSec = shotStartSec;
            inShot = false;
            dipArmed = true;
        }
    }

    private void clearLog() {
        shots.clear();
        shotCount = 0;
        lastShotSec = Double.NaN;
    }

    // ---- Telemetry: live info on top, the record line at the bottom ---------------

    private void showTelemetry(double dt) {
        double target = flywheelOn ? value[Knob.FLYWHEEL.ordinal()] : 0;
        telemetry.addLine("POLLEN ONLY -- nectar guard OFF");
        telemetry.addData("Flywheel", "%s  L %.0f / R %.0f  (target %.0f)%s",
                flywheelOn ? (isAtSpeed() ? "AT SPEED" : "spinning") : "off",
                leftRpm, rightRpm, target,
                flywheelOn && Math.abs(leftRpm - rightRpm) > ShooterLogic.DISAGREE_RPM ? "  WHEELS DISAGREE" : "");
        telemetry.addData("Indexer", "%.0f RPM actual", collection.getIndexerRPM());
        telemetry.addData("Turret", "%s %+.0f deg%s | %s", turret.getState(), turret.getAngle(),
                turretParked ? " PARKED" : turretManual ? " MANUAL" : "", aim.getStatus());
        telemetry.addData("Aim at", targetCell.displayName);
        if (!deflector.isZeroed()) {
            telemetry.addLine(">>> Deflector NOT zeroed -- arc isn't moving. Run Deflector Zero.");
        } else {
            telemetry.addData("Deflector", "%.1f deg (target %.1f%s)", deflector.getAngle(),
                    deflector.getTarget(), deflector.isOnTarget() ? "" : ", moving");
        }

        // Shot log
        telemetry.addLine("");
        telemetry.addLine("SHOTS (newest first)  dip / recover / gap      [gamepad1 B clears]");
        if (shots.isEmpty()) telemetry.addLine("  none yet");
        double dipMin = Double.MAX_VALUE, dipMax = 0, recMin = Double.MAX_VALUE, recMax = 0;
        for (Shot s : shots) {
            telemetry.addLine(String.format("  #%d  %.0f RPM / %.2f s / %s", s.n, s.dipRpm, s.recoverSec,
                    Double.isNaN(s.gapSec) ? "-" : String.format("%.2f s%s", s.gapSec, s.firstOfBurst ? " (fed)" : "")));
            dipMin = Math.min(dipMin, s.dipRpm);  dipMax = Math.max(dipMax, s.dipRpm);
            recMin = Math.min(recMin, s.recoverSec); recMax = Math.max(recMax, s.recoverSec);
        }
        if (shots.size() >= 2) {
            telemetry.addLine(String.format("  spread over %d: dip %.0f-%.0f (%.0f)  recover %.2f-%.2f s",
                    shots.size(), dipMin, dipMax, dipMax - dipMin, recMin, recMax));
        }

        // Knobs
        telemetry.addLine("");
        telemetry.addLine("TUNE (gamepad1: D-pad U/D pick, R/L step, hold A fine, Y reset)");
        for (Knob k : Knob.values()) {
            telemetry.addLine(String.format("%s %-9s %s %s   (step %s)%s",
                    k.ordinal() == selected ? ">" : " ", k.label, fmt(k, value[k.ordinal()]), k.unit,
                    fmt(k, k.step), k == Knob.PIVOT ? "  higher = tighter" : ""));
        }

        // The record line
        telemetry.addLine("");
        telemetry.addLine("==== RECORD THIS ====");
        telemetry.addLine(String.format("%s | Coll %s | Idx %s | Fly %s | Defl %s",
                robotName(), fmt(Knob.COLLECTOR, value[Knob.COLLECTOR.ordinal()]),
                fmt(Knob.INDEXER, value[Knob.INDEXER.ordinal()]),
                fmt(Knob.FLYWHEEL, value[Knob.FLYWHEEL.ordinal()]),
                fmt(Knob.DEFLECTOR, value[Knob.DEFLECTOR.ordinal()])));
        telemetry.addLine(String.format("Pivot %s (%+.3f from stock %.3f)%s",
                fmt(Knob.PIVOT, value[Knob.PIVOT.ordinal()]),
                value[Knob.PIVOT.ordinal()] - start[Knob.PIVOT.ordinal()], start[Knob.PIVOT.ordinal()],
                pivotGapText()));
        telemetry.addLine(String.format("Range %s | Loop %.0f ms", rangeText(), dt * 1000));
        telemetry.update();
    }

    private String pivotGapText() {
        if (Double.isNaN(PIVOT_GAP_AT_STOCK_IN) || Double.isNaN(PIVOT_IN_PER_UNIT)) return "";
        double gap = PIVOT_GAP_AT_STOCK_IN
                - (value[Knob.PIVOT.ordinal()] - start[Knob.PIVOT.ordinal()]) * PIVOT_IN_PER_UNIT;
        return String.format(" | gap ~%.2f in, squeeze %.2f in", gap, POLLEN_DIAMETER_IN - gap);
    }

    /** Camera range to the nearest tag of the cell we're aiming at. */
    private String rangeText() {
        double best = Double.NaN;
        int bestId = -1;
        for (AprilTagDetection d : camera.getDetections()) {
            if (d.metadata == null || d.ftcPose == null) continue;
            for (int id : targetCell.tagIds) {
                if (d.id == id && (Double.isNaN(best) || d.ftcPose.range < best)) {
                    best = d.ftcPose.range;
                    bestId = d.id;
                }
            }
        }
        return Double.isNaN(best) ? "no tag in view" : String.format("%.1f in to tag %d", best, bestId);
    }

    // ---- Helpers ----------------------------------------------------------------

    private static String fmt(Knob k, double v) { return String.format(k.format, v); }

    private static String robotName() {
        try { return WhichRobot.number(); } catch (IllegalStateException e) { return "robot ?"; }
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * RobotHardware.TICKS_PER_MOTOR_REV * Shooter.SHOOTER_GEAR_RATIO / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return tps / (RobotHardware.TICKS_PER_MOTOR_REV * Shooter.SHOOTER_GEAR_RATIO) * 60.0;
    }

    private static int[] hiveIdsIn(List<AprilTagDetection> detections) {
        int n = 0;
        int[] ids = new int[detections.size()];
        for (AprilTagDetection d : detections) if (d.metadata != null) ids[n++] = d.id;
        return java.util.Arrays.copyOf(ids, n);
    }

    private void setTarget(FieldAprilTags.Cell cell) {
        if (cell == targetCell) return;
        targetCell = cell;
        aim.setTargetTagIds(cell.tagIds);
        aim.resetPhase();
    }

    private FieldAprilTags.Cell cellFor(boolean scoring) {
        if (alliance.get() == Alliance.RED) {
            return scoring ? FieldAprilTags.Cell.RED_SCORING : FieldAprilTags.Cell.RED_AUDIENCE;
        }
        return scoring ? FieldAprilTags.Cell.BLUE_SCORING : FieldAprilTags.Cell.BLUE_AUDIENCE;
    }

    @Override
    public void stop() {
        if (hw != null) {
            hw.shooterLeft.setPower(0);
            hw.shooterRight.setPower(0);
        }
        if (collection != null) collection.stopAll();
        if (turret != null) turret.stop();
        if (deflector != null) deflector.stop();
        if (camera != null) camera.close();
    }
}
