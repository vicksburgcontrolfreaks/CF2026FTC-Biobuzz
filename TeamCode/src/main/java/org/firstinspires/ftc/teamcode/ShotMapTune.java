package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Auton.AutoMenu;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldAprilTags;
import org.firstinspires.ftc.teamcode.field.HiveGeometry;
import org.firstinspires.ftc.teamcode.field.WallStart;
import org.firstinspires.ftc.teamcode.geometry.FieldCentric;
import org.firstinspires.ftc.teamcode.geometry.TurretGeometry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.ShooterLogic;
import org.firstinspires.ftc.teamcode.shooting.ShotMap;
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Ladder;
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Outcome;
import org.firstinspires.ftc.teamcode.subsystems.CollectionSystem;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretAim;
import org.firstinspires.ftc.teamcode.subsystems.TurretDirector;
import org.firstinspires.ftc.teamcode.vision.TurretCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/*
 * Shot Map: find the flywheel RPM that scores from each distance, and how the
 * turret angle shifts it, and LOG EVERY BALL so nothing has to be written down.
 *
 * WHAT IT IS FOR (Shane, 2026-11-13)
 *   1. A ShotTable for this robot: distance to the hive -> RPM. Shoot from 4-6
 *      distances with the turret near 0.
 *   2. The flywheel-offset correction. On 6494 the flywheels sit ~0.5-1 in left
 *      of the turret's centre, so where the ball lands on the deflector depends on
 *      the TURRET ANGLE. At one or two distances, rotate the robot IN PLACE so the
 *      turret sits at about -90, -45, 0, +45, +90, and map each. The fit line
 *      turns that into a few numbers (see ShotMap). On 5618 the turret is centred
 *      over the flywheels, so its fit should come out near zero -- confirm it.
 *
 * HOW A SPOT GOES
 *   Park somewhere, fire ONE ball, press what it did. After BALLS_PER_RUNG balls
 *   at one RPM the ladder moves on by itself (FOLLOW_SUGGESTION): up a rung if
 *   that rung was mostly short, down if mostly long, outward if mostly hits.
 *   Keep going until the screen says DONE, then press B to save the spot and
 *   drive to the next one. One ball per press: if two go, press for the first.
 *
 *   BALLS ARE RANDOM, so no single ball decides anything. Every RPM gets several
 *   balls, and the answer is the BALANCE POINT where longs and shorts cancel,
 *   using every ball. The screen also shows the best hit rate (low everywhere =
 *   the balls/mechanism are the limit, not the RPM) and the transition width
 *   (how much RPM error the shot forgives). See ShotMap.
 *
 * WHERE THE ROBOT IS
 *   Odometry, started from the wall start picked in the INIT menu. Park on it
 *   before Start. Driving between spots drifts the pose a little; to re-sync,
 *   park back on the same start and press BACK on gamepad 1. The "Pose" line
 *   shows the camera's opinion: a trim of several degrees, or NOT TRUSTED, means
 *   the pose has drifted and the distances being logged are off -- re-sync.
 *
 * WHAT STAYS FIXED: pivot at POLLEN_POSITION, deflector at DEFAULT_SHOT_DEG,
 * indexer at FEED_RPM. RPM is the only knob (Shane's call, 2026-11-13). POLLEN
 * ONLY -- there is no nectar guard here.
 *
 * CONTROLS -- either gamepad, so one person can run it solo
 *   Left stick / right stick X   drive (gamepad 1 only, field-centric)
 *   RT (hold)      feed: indexer + collector forward
 *   Y              flywheel on / off
 *   A              HIT
 *   D-pad Down     SHORT          D-pad Up     LONG
 *   D-pad Left     went LEFT      D-pad Right  went RIGHT
 *   RB / LB        RPM up / down by RPM_STEP
 *   LT + RB / LB   aim trim left / right by AIM_STEP_DEG
 *   X              undo the last press
 *   B              save this spot, start the next
 *   BACK (gp1)     "I'm parked on the start spot" -- re-sync the pose
 *
 * THE LOG: /sdcard/FIRST/shotmap/shotmap-<robot>-<date>.csv, one "shot" row per
 * press and one "spot" row per B. Pull it with
 *     adb pull /sdcard/FIRST/shotmap
 * or from the Control Hub's file manager. The screen shows the path.
 */
@Configurable
@TeleOp(name = "Shot Map", group = "Tune")
public class ShotMapTune extends OpMode {

    // ---- Knobs ---------------------------------------------------------------
    /** Manual RPM step for the bumpers. */
    public static double RPM_STEP = 10;   // fine: shots live at ~1200-1300 RPM (Shane, 2026-11-13)
    /** Ladder rung spacing: the biggest jump Shane wants at ~1200-1300 RPM. */
    public static double RPM_RUNG = 25;
    /**
     * Balls on each rung before the ladder moves. 4 found the centre within 15 RPM
     * in 191/200 laptop runs with scatter as wide as the band: median 16 balls a
     * spot, 20 at the 90th percentile. Drop to 3 once you have seen how noisy the shooter really is.
     */
    public static int BALLS_PER_RUNG = 4;
    public static double MIN_RPM = 500;
    /** Pollen Shot Tune's ceiling. Raise only after checking the wheels hold it. */
    public static double MAX_RPM = 2500;

    public static double AIM_STEP_DEG = 0.5;
    /** Aim ladder rung spacing. It only moves when a rung is mostly left or right. */
    public static double AIM_RUNG_DEG = 1.0;

    /** Move to the suggested RPM / trim automatically after each press. */
    public static boolean FOLLOW_SUGGESTION = true;

    /** Spots closer than this in turret angle count as "the same angle" for the fit. */
    public static double FIT_ANGLE_BIN_DEG = 15;

    // ---- Driving / misc ------------------------------------------------------
    private static final double TRIGGER_HELD = 0.5;
    private static final double COLLECTOR_RPM = 200;

    // ---- INIT menu ------------------------------------------------------------
    private final AutoMenu menu = new AutoMenu();
    private final AutoMenu.Choice<WallStart> startSpot = menu.add("Start on", WallStart.values());
    private final AutoMenu.Choice<TurretDirector.AimSource> aimSource =
            menu.add("Aim by", TurretDirector.AimSource.CAMERA, TurretDirector.AimSource.ODOMETRY)
                    .summary("aim %s")
                    .allowWhen(TurretDirector.AimSource::isAvailable);

    // ---- Hardware ----------------------------------------------------------------
    private RobotHardware hw;
    private Follower follower;
    private CollectionSystem collection;
    private ShooterPivot pivot;
    private Deflector deflector;
    private Turret turret;
    private TurretCamera camera;
    private TurretDirector aim;
    private FieldAprilTags.Cell targetCell;
    private Alliance alliance;
    private String robot = "robot ?";

    // ---- Flywheel ------------------------------------------------------------------
    private boolean flywheelOn = true;
    private double rpm;
    private double leftRpm, rightRpm;

    // ---- Shot detection (same rule as Pollen Shot Tune) -----------------------------
    // A shot = the slower wheel sags DIP_RPM below target after having been back
    // within TOLERANCE_RPM. The speeds JUST BEFORE the sag are what the ball saw.
    private boolean dipArmed = false, inShot = false;
    private double peakDrop, steadyLeft = Double.NaN, steadyRight = Double.NaN;
    private int inBurst = 0;
    private boolean feeding = false;
    /** The last detected ball, waiting for its outcome press. Null = none since the last press. */
    private double[] pendingShot = null;   // {left, right, dip, inBurst}
    private double lastTargetRpm = -1;

    // ---- The map -------------------------------------------------------------------
    private double aimTrim = 0;
    private double baseAimBias = Double.NaN;   // TurretAim.AIM_BIAS_DEG before we touched it; NaN = never started
    private int spotNumber = 1, shotNumber = 0;
    private final List<Press> spotPresses = new ArrayList<>();
    private Ladder rpmLadder = newRpmLadder(), aimLadder = newAimLadder();
    private final List<Spot> spots = new ArrayList<>();

    private static Ladder newRpmLadder() { return new Ladder(RPM_RUNG, BALLS_PER_RUNG, true); }
    private static Ladder newAimLadder() { return new Ladder(AIM_RUNG_DEG, BALLS_PER_RUNG, false); }
    private String lastNote = "fire one ball, then press what it did";

    private static final class Press {
        final double rpm, trim, distance, turretDeg;
        final Outcome outcome;
        Press(double rpm, double trim, double distance, double turretDeg, Outcome outcome) {
            this.rpm = rpm; this.trim = trim; this.distance = distance;
            this.turretDeg = turretDeg; this.outcome = outcome;
        }
    }

    private static final class Spot {
        final int n; final double distance, turretDeg, rpmBalance, rpmSigma, rpmWidth, bestHit, aimCentre;
        final boolean rpmUsable;
        Spot(int n, double distance, double turretDeg, Ladder rpm, Ladder aim) {
            this.n = n; this.distance = distance; this.turretDeg = turretDeg;
            this.rpmBalance = rpm.balance();
            this.rpmSigma = rpm.balanceSigma();
            this.rpmWidth = rpm.transitionWidth();
            this.bestHit = rpm.bestHitRate();
            this.rpmUsable = rpm.isBounded() && !rpm.isNoisy() && !Double.isNaN(rpmBalance);
            this.aimCentre = aim.estimate();
        }
    }

    // ---- Log file ------------------------------------------------------------------
    private PrintWriter log;
    private String logPath = "(not open)";
    private String logError = null;

    // ================================================================================

    @Override
    public void init() {
        RobotProfile.install();
        aimSource.set(TurretDirector.AimSource.fromDefault());
        try { robot = WhichRobot.number(); } catch (IllegalStateException e) { robot = "robot ?"; }

        hw = new RobotHardware();
        hw.init(hardwareMap, false);   // TurretCamera owns the webcam
        follower = Constants.createFollower(hardwareMap);
        collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);
        pivot = new ShooterPivot(hw.shooterPivot);
        deflector = new Deflector(hardwareMap);
        turret = hw.turret;
        camera = new TurretCamera(hardwareMap);

        for (DcMotorEx m : new DcMotorEx[]{hw.shooterLeft, hw.shooterRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        rpm = Launcher.defaultFlywheelRpm();
        // Bench tuner, never run in a match: homing the deflector in INIT is fine
        // here (same reasoning as Pollen Shot Tune).
        if (deflector.isPresent() && !deflector.isZeroed()) deflector.startHoming();
    }

    @Override
    public void init_loop() {
        menu.update(gamepad1);
        camera.update();
        deflector.update(true);
        telemetry.addLine("SHOT MAP -- pollen only, NO nectar guard");
        menu.show(telemetry);
        telemetry.addData("Robot", robot);
        telemetry.addLine("PARK ON THE START SPOT before Start: " + startSpot.get().label);
        telemetry.addLine("  side on the wall, back edge on the tile seam, facing the other alliance");
        if (deflector.isPresent() && !deflector.isZeroed() && !deflector.isHoming()) {
            telemetry.addLine(">>> Deflector NOT zeroed -- run Deflector Auto Home.");
        }
        if (!TurretGeometry.forRobot().isConfigured()) {
            telemetry.addLine(">>> Turret offset not measured on this robot: distances");
            telemetry.addLine("    are from the robot centre, not the turret (approx).");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        aimSource.get().apply();
        WallStart s = startSpot.get();
        alliance = s.alliance;
        follower.setStartingPose(s.pose);
        targetCell = cellFor(true);
        aim = new TurretDirector(targetCell.tagIds);
        if (!turret.isHomed() && Turret.isMeasured()) turret.startHoming();
        pivot.setPosition(ShooterPivot.POLLEN_POSITION);
        if (deflector.isZeroed()) deflector.goTo(Deflector.DEFAULT_SHOT_DEG);
        baseAimBias = TurretAim.AIM_BIAS_DEG;
        openLog();
    }

    @Override
    public void loop() {
        follower.update();
        camera.update();
        drive();
        controls();
        runFlywheel();
        runFeed();
        aimTurret();
        detectShots();
        deflector.update(true);
        showTelemetry();
    }

    // ---- Driving ----------------------------------------------------------------------

    private void drive() {
        if (gamepad1.backWasPressed()) {
            follower.setPose(startSpot.get().pose);
            aim.resetPhase();
            lastNote = "pose re-synced to " + startSpot.get().label;
            gamepad1.rumble(300);
        }
        double[] r = FieldCentric.toRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                follower.getPose().getHeading(), alliance.awayFromDriversHeading());
        double forward = r[0], right = r[1], turn = gamepad1.right_stick_x;
        double den = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);
        hw.lf.setPower((forward + right + turn) / den);
        hw.rf.setPower((forward - right - turn) / den);
        hw.lr.setPower((forward - right + turn) / den);
        hw.rr.setPower((forward + right - turn) / den);
    }

    // ---- Buttons ----------------------------------------------------------------------

    private void controls() {
        // Read every wasPressed() exactly ONCE per loop: each call clears itself.
        boolean a = gamepad1.aWasPressed() | gamepad2.aWasPressed();
        boolean down = gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed();
        boolean up = gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed();
        boolean rb = gamepad1.rightBumperWasPressed() | gamepad2.rightBumperWasPressed();
        boolean lb = gamepad1.leftBumperWasPressed() | gamepad2.leftBumperWasPressed();
        boolean x = gamepad1.xWasPressed() | gamepad2.xWasPressed();
        boolean b = gamepad1.bWasPressed() | gamepad2.bWasPressed();
        boolean y = gamepad1.yWasPressed() | gamepad2.yWasPressed();
        boolean ltHeld = held(gamepad1.left_trigger) || held(gamepad2.left_trigger);

        if (a) record(Outcome.HIT);
        if (down) record(Outcome.SHORT);
        if (up) record(Outcome.LONG);
        if (left) record(Outcome.LEFT);
        if (right) record(Outcome.RIGHT);
        if (x) undo();
        if (b) finishSpot();
        if (y) flywheelOn = !flywheelOn;

        if (ltHeld) {
            if (rb) setTrim(aimTrim + AIM_STEP_DEG);
            if (lb) setTrim(aimTrim - AIM_STEP_DEG);
        } else {
            if (rb) setRpm(rpm + RPM_STEP);
            if (lb) setRpm(rpm - RPM_STEP);
        }
    }

    private static boolean held(float trigger) {
        return trigger > TRIGGER_HELD;
    }

    private void setRpm(double v) {
        rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, Math.round(v)));
    }

    /** Trim + = turret further counter-clockwise (left). Applied through TurretAim.AIM_BIAS_DEG. */
    private void setTrim(double v) {
        aimTrim = Math.round(v * 100) / 100.0;
        TurretAim.AIM_BIAS_DEG = baseAimBias + aimTrim;
        aim.resetPhase();   // camera aiming re-commits with the new bias
    }

    // ---- Recording --------------------------------------------------------------------

    private void record(Outcome outcome) {
        Pose p = follower.getPose();
        double distance = distance(p);
        double turretDeg = turret.getAngle();

        double left = leftRpm, right = rightRpm, dip = Double.NaN;
        int burst = 0;
        if (pendingShot != null) {
            left = pendingShot[0];
            right = pendingShot[1];
            dip = pendingShot[2];
            burst = (int) pendingShot[3];
            pendingShot = null;
        }

        shotNumber++;
        spotPresses.add(new Press(rpm, aimTrim, distance, turretDeg, outcome));
        rebuildLadders();
        write(ShotMap.shotRow(getRuntime(), robot, spotNumber, shotNumber, startSpot.get().label,
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()), distance, turretDeg, rpm,
                left, right, burst, dip, aimTrim, deflector.getAngle(), aim.isPoseTrusted(), outcome));

        String was = String.format(Locale.US, "#%d %s at %.0f", shotNumber, outcome, rpm);
        if (FOLLOW_SUGGESTION) {
            // Each ladder only moves once its current rung has BALLS_PER_RUNG balls,
            // so no single ball moves anything. A sideways ball says nothing about
            // RPM and is not counted there; every ball counts for the aim.
            double nextRpm = rpmLadder.next(rpm);
            double nextTrim = aimLadder.next(aimTrim);
            if (nextRpm != rpm) was += String.format(Locale.US, " -> rung %.0f", nextRpm);
            if (nextTrim != aimTrim) was += String.format(Locale.US, " -> trim %+.1f", nextTrim);
            setRpm(nextRpm);
            if (nextTrim != aimTrim) setTrim(nextTrim);
        }
        if (rpmLadder.isDone()) was += "  DONE -> B";
        lastNote = was + (burst == 0 ? "  (no dip seen)" : "");
    }

    private void rebuildLadders() {
        rpmLadder = newRpmLadder();
        aimLadder = newAimLadder();
        for (Press pr : spotPresses) {
            if (pr.outcome.forRpm() != null) rpmLadder.record(pr.rpm, pr.outcome.forRpm(), pr.outcome.letter());
            aimLadder.record(pr.trim, pr.outcome.forAim(), pr.outcome.letter());
        }
    }

    private void undo() {
        if (spotPresses.isEmpty()) {
            lastNote = "nothing to undo in this spot";
            return;
        }
        Press gone = spotPresses.remove(spotPresses.size() - 1);
        rebuildLadders();
        // The file is append-only, so the undo is a row of its own; anything
        // reading the log drops the shot it names.
        write("undo,," + robot + "," + spotNumber + "," + shotNumber);
        setRpm(gone.rpm);
        setTrim(gone.trim);
        lastNote = String.format(Locale.US, "undid #%d (%s) -- back to %.0f", shotNumber, gone.outcome, gone.rpm);
    }

    private void finishSpot() {
        if (spotPresses.isEmpty()) {
            lastNote = "no shots in this spot yet";
            return;
        }
        double d = 0, t = 0;
        for (Press pr : spotPresses) { d += pr.distance; t += pr.turretDeg; }
        d /= spotPresses.size();
        t /= spotPresses.size();
        Spot s = new Spot(spotNumber, d, t, rpmLadder, aimLadder);
        spots.add(s);
        write(ShotMap.spotRow(robot, spotNumber, d, t, spotPresses.size(), rpmLadder, aimLadder));
        lastNote = String.format(Locale.US, "saved spot %d: %.0f in, turret %+.0f -> %s", spotNumber, d, t,
                s.rpmUsable ? String.format(Locale.US, "%.0f RPM", s.rpmBalance)
                        : rpmLadder.isNoisy() ? "NOISY, not used in the fit" : "NOT BOUNDED, not used in the fit");
        // Start the next spot from this one's answer, snapped to the rung grid:
        // the next distance is usually close.
        if (!Double.isNaN(s.rpmBalance)) setRpm(Math.round(s.rpmBalance / RPM_RUNG) * RPM_RUNG);
        spotNumber++;
        spotPresses.clear();
        rebuildLadders();
        gamepad1.rumble(200);
    }

    // ---- Shooter --------------------------------------------------------------------------

    private void runFlywheel() {
        double target = flywheelOn ? rpm : 0;
        if (target != lastTargetRpm) {   // a new target isn't a shot: wait for the wheels to settle
            dipArmed = false;
            inShot = false;
            lastTargetRpm = target;
        }
        if (target <= 0) {
            hw.shooterLeft.setPower(0);   // coast, don't brake a flywheel
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
    }

    private void runFeed() {
        boolean feed = held(gamepad1.right_trigger) || held(gamepad2.right_trigger);
        if (feed && !feeding) inBurst = 0;
        feeding = feed;
        if (feed) {
            collection.setIndexerRPM(CollectionSystem.FEED_RPM);
            collection.setCollectorRPM(COLLECTOR_RPM);
        } else {
            collection.stopIndexer();
            collection.stopCollector();
        }
    }

    private void detectShots() {
        double target = lastTargetRpm;
        if (target <= 0) return;
        double drop = target - Math.min(leftRpm, rightRpm);
        if (!inShot) {
            if (drop < ShooterLogic.TOLERANCE_RPM) {
                dipArmed = true;
                steadyLeft = leftRpm;
                steadyRight = rightRpm;
            }
            if (dipArmed && drop >= ShooterLogic.DIP_RPM) {
                inShot = true;
                dipArmed = false;
                peakDrop = drop;
            }
            return;
        }
        peakDrop = Math.max(peakDrop, drop);
        if (drop < ShooterLogic.TOLERANCE_RPM) {
            inBurst++;
            pendingShot = new double[]{steadyLeft, steadyRight, peakDrop, inBurst};
            inShot = false;
            dipArmed = true;
        }
    }

    // ---- Turret ---------------------------------------------------------------------------

    private void aimTurret() {
        List<AprilTagDetection> detections = camera.getDetections();
        setTarget(HiveGeometry.cameraCell(alliance, hiveIdsIn(detections), targetCell));
        double[] opening = HiveGeometry.aimPointFor(targetCell);
        aim.update(detections, turret, follower.getPose(), opening[0], opening[1],
                turret.isHomed(), getRuntime());
        turret.update();
    }

    /** Flat inches from the turret axis to the cell's opening -- what ShotTable is keyed on. */
    private double distance(Pose p) {
        double[] opening = HiveGeometry.aimPointFor(targetCell);
        TurretGeometry g = TurretGeometry.forRobot();
        if (!g.isConfigured()) {
            g = new TurretGeometry(0, 0, TurretGeometry.TURRET_ZERO_DEG, TurretGeometry.TURRET_CCW_SIGN);
        }
        return g.distanceTo(p, opening[0], opening[1]);
    }

    // ---- Telemetry ------------------------------------------------------------------------

    private void showTelemetry() {
        Pose p = follower.getPose();
        telemetry.addLine(String.format(Locale.US, "SHOT MAP %s | spot %d, %d balls | %s",
                robot, spotNumber, spotPresses.size(), targetCell.displayName));
        telemetry.addLine(">> " + lastNote);

        telemetry.addData("DIST", "%.1f in   turret %+.0f deg%s", distance(p), turret.getAngle(),
                turret.isHomed() ? "" : "  (NOT HOMED)");
        telemetry.addData("Pose", "(%.0f, %.0f) %.0f deg | %s%s", p.getX(), p.getY(),
                Math.toDegrees(p.getHeading()),
                aim.isPoseTrusted() ? "trusted" : "NOT TRUSTED -- re-sync (BACK)",
                aim.hasCameraFix() ? String.format(Locale.US, ", cam trim %+.1f", aim.getTrimDeg()) : "");

        boolean disagree = flywheelOn && Math.abs(leftRpm - rightRpm) > ShooterLogic.DISAGREE_RPM;
        telemetry.addData("RPM", "%.0f  (L %.0f / R %.0f)%s%s", rpm, leftRpm, rightRpm,
                flywheelOn ? (atSpeed() ? " AT SPEED" : " spinning") : " OFF (Y)",
                disagree ? "  WHEELS DISAGREE" : "");
        showLadder();
        double est = aimLadder.estimate();
        telemetry.addData("Aim trim", "%+.2f deg | est %s%s", aimTrim,
                Double.isNaN(est) ? "--" : String.format(Locale.US, "%+.1f", est),
                trimReachesTurret() ? "" : "  (NOT APPLIED: " + aim.getSource() + ")");

        telemetry.addLine("");
        telemetry.addLine("SPOTS  dist / turret -> balance RPM, width, best hit / trim");
        for (int i = Math.max(0, spots.size() - 5); i < spots.size(); i++) {
            Spot s = spots.get(i);
            telemetry.addLine(String.format(Locale.US, "  %d: %.0f in %+4.0f -> %s / %s", s.n,
                    s.distance, s.turretDeg,
                    Double.isNaN(s.rpmBalance) ? "--"
                            : String.format(Locale.US, "%s%.0f, w %s, hit %.0f%%", s.rpmUsable ? "" : "~",
                            s.rpmBalance, Double.isNaN(s.rpmWidth) ? "?" : String.format(Locale.US, "%.0f", s.rpmWidth),
                            s.bestHit * 100),
                    Double.isNaN(s.aimCentre) ? "--" : String.format(Locale.US, "%+.1f", s.aimCentre)));
        }
        if (spots.isEmpty()) telemetry.addLine("  none saved yet (B saves one)");
        telemetry.addLine(fitText());

        telemetry.addLine("");
        telemetry.addLine("A hit | Dpad dn short, up long, L/R left/right | RB/LB rpm | LT+RB/LB trim");
        telemetry.addLine("X undo | B save spot | Y flywheel | RT feed | BACK re-sync pose");
        telemetry.addLine(logError != null ? ">>> LOG FAILED: " + logError : "Log: " + logPath);
        telemetry.update();
    }

    /**
     * The RPM ladder, highest rung first:
     *     1275    LLHL   hit 25%  +0.75
     *   > 1250    HHSH   hit 75%  -0.25    (shooting here)
     * then the balance point, the width, and the best hit rate so far.
     */
    private void showLadder() {
        List<Ladder.Rung> rungs = rpmLadder.rungs();
        boolean hereListed = false;
        for (int i = rungs.size() - 1; i >= 0; i--) {
            Ladder.Rung r = rungs.get(i);
            boolean here = Math.abs(r.value - rpm) < 0.5;
            hereListed |= here;
            telemetry.addLine(String.format(Locale.US, "  %s %4.0f   %-6s hit %3.0f%%  %+.2f%s",
                    here ? ">" : " ", r.value, r.sequence(), r.hitRate() * 100, r.score(),
                    r.n() < BALLS_PER_RUNG ? String.format(Locale.US, "  (%d/%d)", r.n(), BALLS_PER_RUNG) : ""));
        }
        if (!hereListed) {
            telemetry.addLine(String.format(Locale.US, "  > %4.0f   (no balls yet)", rpm));
        }
        double b = rpmLadder.balance();
        double w = rpmLadder.transitionWidth();
        telemetry.addLine(String.format(Locale.US, "  balance %s | width %s | best hit %.0f%%%s",
                Double.isNaN(b) ? "--" : String.format(Locale.US, "%.0f", b),
                Double.isNaN(w) ? "--" : String.format(Locale.US, "%.0f RPM", w),
                rpmLadder.bestHitRate() * 100,
                rpmLadder.isNoisy() ? "  NOISY: rungs contradict -- B saves it as unusable"
                        : rpmLadder.isDone() ? "  DONE -> B" : ""));
    }

    private boolean atSpeed() {
        return flywheelOn && Math.abs(leftRpm - rpm) <= ShooterLogic.TOLERANCE_RPM
                && Math.abs(rightRpm - rpm) <= ShooterLogic.TOLERANCE_RPM;
    }

    /**
     * The trim goes in through TurretAim.AIM_BIAS_DEG, which the camera uses
     * directly and odometry uses only through the camera trim. Pure odometry with
     * no camera trim ignores it, and the screen should say so.
     */
    private boolean trimReachesTurret() {
        return !aim.isUsingOdometry() || TurretDirector.USE_CAMERA_TRIM;
    }

    /**
     * The flywheel-offset fit, with a +- (1 sigma) on each term. A term inside
     * about 2x its +- is not distinguishable from noise -- on 5618, with the turret
     * centred, that is the expected answer.
     *
     * RPM: usable (bounded, not noisy) spots, weighted by how sure each balance is;
     * needs 5+ over 3+ turret angles, since it also has a distance term. Aim: 4+
     * spots over 3+ angles.
     */
    private String fitText() {
        List<Spot> r = new ArrayList<>();
        List<Spot> a = new ArrayList<>();
        for (Spot s : spots) {
            if (s.rpmUsable) r.add(s);
            if (!Double.isNaN(s.aimCentre)) a.add(s);
        }
        StringBuilder sb = new StringBuilder("FIT  ");
        ShotMap.Fit c = null;
        if (r.size() >= 5 && angles(r) >= 3) {
            double[][] rows = new double[r.size()][];
            double[] y = new double[r.size()];
            double[] w = new double[r.size()];
            for (int i = 0; i < r.size(); i++) {
                rows[i] = ShotMap.rpmRow(r.get(i).distance, r.get(i).turretDeg);
                y[i] = r.get(i).rpmBalance;
                w[i] = 1 / Math.max(1, r.get(i).rpmSigma * r.get(i).rpmSigma);
            }
            c = ShotMap.fit(rows, y, w);
        }
        sb.append(c == null
                ? String.format(Locale.US, "rpm: need 5 usable spots, 3 angles (%d, %d)", r.size(), angles(r))
                : String.format(Locale.US, "rpm cos %s sin %s (%.1f/in)",
                        pm(c.coef[2], c.sigma[2], "%+.0f"), pm(c.coef[3], c.sigma[3], "%+.0f"), c.coef[1]));
        ShotMap.Fit k = null;
        if (a.size() >= 4 && angles(a) >= 3) {
            double[][] rows = new double[a.size()][];
            double[] y = new double[a.size()];
            for (int i = 0; i < a.size(); i++) {
                rows[i] = ShotMap.aimRow(a.get(i).turretDeg);
                y[i] = a.get(i).aimCentre;
            }
            k = ShotMap.fit(rows, y, null);
        }
        sb.append(k == null ? " | aim: need 4 spots, 3 angles"
                : String.format(Locale.US, " | aim %s, cos %s sin %s",
                        pm(k.coef[0], k.sigma[0], "%+.1f"), pm(k.coef[1], k.sigma[1], "%+.1f"),
                        pm(k.coef[2], k.sigma[2], "%+.1f")));
        return sb.toString();
    }

    private static String pm(double v, double sigma, String fmt) {
        String s = String.format(Locale.US, fmt, v);
        return Double.isNaN(sigma) ? s : s + "+-" + String.format(Locale.US, "%.1f", sigma);
    }

    private static int angles(List<Spot> spots) {
        double[] t = new double[spots.size()];
        for (int i = 0; i < t.length; i++) t[i] = spots.get(i).turretDeg;
        return ShotMap.distinctAngles(t, FIT_ANGLE_BIN_DEG);
    }

    // ---- Log file --------------------------------------------------------------------------

    private void openLog() {
        try {
            File dir = new File(AppUtil.ROOT_FOLDER, "shotmap");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("can't make " + dir);
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
            File f = new File(dir, "shotmap-" + robot + "-" + stamp + ".csv");
            log = new PrintWriter(new FileWriter(f, true));
            logPath = f.getAbsolutePath();
            log.println(ShotMap.SHOT_HEADER);
            log.println(ShotMap.SPOT_HEADER);
            log.flush();
        } catch (IOException | RuntimeException e) {
            log = null;
            logError = e.getMessage();
        }
    }

    /** Flushed every row: a dead battery mid-session must not cost the data. */
    private void write(String row) {
        if (log == null) return;
        log.println(row);
        log.flush();
        if (log.checkError()) logError = "write failed";
    }

    // ---- Helpers ---------------------------------------------------------------------------

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
        return Arrays.copyOf(ids, n);
    }

    private void setTarget(FieldAprilTags.Cell cell) {
        if (cell == targetCell) return;
        targetCell = cell;
        aim.setTargetTagIds(cell.tagIds);
        aim.resetPhase();
    }

    private FieldAprilTags.Cell cellFor(boolean scoring) {
        if (alliance == Alliance.RED) {
            return scoring ? FieldAprilTags.Cell.RED_SCORING : FieldAprilTags.Cell.RED_AUDIENCE;
        }
        return scoring ? FieldAprilTags.Cell.BLUE_SCORING : FieldAprilTags.Cell.BLUE_AUDIENCE;
    }

    @Override
    public void stop() {
        // Put the camera's bias back: it is a static, and the next op mode in this
        // power-up would otherwise aim with our trim baked in.
        if (!Double.isNaN(baseAimBias)) TurretAim.AIM_BIAS_DEG = baseAimBias;
        if (hw != null) {
            hw.shooterLeft.setPower(0);
            hw.shooterRight.setPower(0);
        }
        if (collection != null) collection.stopAll();
        if (turret != null) turret.stop();
        if (deflector != null) deflector.stop();
        if (camera != null) camera.close();
        if (log != null) log.close();
    }
}
