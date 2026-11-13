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
import org.firstinspires.ftc.teamcode.shooting.ShotMap.Bracket;
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
 *   Park somewhere, fire ONE ball, press what it did. The RPM moves to the next
 *   value worth trying by itself (FOLLOW_SUGGESTION). Keep going until the RPM
 *   line says DONE, then press B to save the spot and drive to the next one.
 *   One ball per press: if two go, press for the first and watch the second.
 *
 *   It hunts for the EDGES of the hit band -- where shots stop landing short and
 *   where they start going long -- not one RPM that happened to score. The middle
 *   is the table value; the width is how forgiving the shot is. See ShotMap.
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
    public static double RPM_STEP = 25;
    /** How far the search steps outward before it has seen an edge. */
    public static double RPM_EXPLORE_STEP = 100;
    /**
     * Stop splitting a gap narrower than this. 10 takes ~16 balls a spot on the
     * laptop tests; 25 roughly halves that. Set it from how wide the bands turn out:
     * no point resolving 10 RPM inside a 300 RPM band.
     */
    public static double RPM_RESOLUTION = 25;
    public static double MIN_RPM = 500;
    /** Pollen Shot Tune's ceiling. Raise only after checking the wheels hold it. */
    public static double MAX_RPM = 2500;

    public static double AIM_STEP_DEG = 0.5;
    /** Trim the search steps when the ball went left/right. */
    public static double AIM_EXPLORE_STEP_DEG = 1.0;
    public static double AIM_RESOLUTION_DEG = 0.5;

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
    private Bracket rpmBracket = new Bracket(), aimBracket = new Bracket();
    private final List<Spot> spots = new ArrayList<>();
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
        final int n; final double distance, turretDeg, rpmCentre, rpmWidth, aimCentre;
        final boolean rpmBounded;
        Spot(int n, double distance, double turretDeg, Bracket rpm, Bracket aim) {
            this.n = n; this.distance = distance; this.turretDeg = turretDeg;
            this.rpmCentre = rpm.centre(); this.rpmWidth = rpm.width();
            this.rpmBounded = rpm.isBounded() && !rpm.isNoisy();
            this.aimCentre = aim.isEmpty() ? Double.NaN : aim.centre();
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
        rebuildBrackets();
        write(ShotMap.shotRow(getRuntime(), robot, spotNumber, shotNumber, startSpot.get().label,
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()), distance, turretDeg, rpm,
                left, right, burst, dip, aimTrim, deflector.getAngle(), aim.isPoseTrusted(), outcome));

        String was = String.format(Locale.US, "#%d %s at %.0f", shotNumber, outcome, rpm);
        if (FOLLOW_SUGGESTION) {
            // RPM follows SHORT/LONG/HIT; the aim follows only LEFT/RIGHT, so the
            // two never move on the same press and every result means one thing.
            if (outcome.forRpm() != null) {
                setRpm(rpmBracket.suggest(rpm, RPM_EXPLORE_STEP, RPM_RESOLUTION));
            }
            if (outcome == Outcome.LEFT || outcome == Outcome.RIGHT) {
                setTrim(aimBracket.suggest(aimTrim, AIM_EXPLORE_STEP_DEG, AIM_RESOLUTION_DEG));
            }
        }
        lastNote = was + (burst == 0 ? "  (no dip seen -- logged current speeds)" : "");
    }

    private void rebuildBrackets() {
        rpmBracket = new Bracket();
        aimBracket = new Bracket();
        for (Press pr : spotPresses) {
            if (pr.outcome.forRpm() != null) rpmBracket.record(pr.rpm, pr.outcome.forRpm());
            if (pr.outcome.forAim() != null) aimBracket.record(pr.trim, pr.outcome.forAim());
        }
    }

    private void undo() {
        if (spotPresses.isEmpty()) {
            lastNote = "nothing to undo in this spot";
            return;
        }
        Press gone = spotPresses.remove(spotPresses.size() - 1);
        rebuildBrackets();
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
        spots.add(new Spot(spotNumber, d, t, rpmBracket, aimBracket));
        write(ShotMap.spotRow(robot, spotNumber, d, t, spotPresses.size(), rpmBracket, aimBracket));
        lastNote = String.format(Locale.US, "saved spot %d: %.0f in, turret %+.0f -> %s",
                spotNumber, d, t, rpmBracket.isBounded()
                        ? String.format(Locale.US, "%.0f RPM", rpmBracket.centre()) : "NOT BOUNDED");
        // Start the next spot from this one's answer: the next distance is usually close.
        if (!Double.isNaN(rpmBracket.centre())) setRpm(rpmBracket.centre());
        spotNumber++;
        spotPresses.clear();
        rebuildBrackets();
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
        telemetry.addLine(String.format(Locale.US, "SHOT MAP %s | spot %d, %d shots | %s",
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
        telemetry.addData("  band", rpmBracket.describe("RPM")
                + (rpmBracket.isDone(RPM_RESOLUTION) ? "  DONE -> B" : ""));
        telemetry.addData("Aim trim", "%+.2f deg | %s%s", aimTrim, aimBracket.describe("deg"),
                trimReachesTurret() ? "" : "  (NOT APPLIED: " + aim.getSource() + ")");

        telemetry.addLine("");
        telemetry.addLine("SPOTS  dist / turret -> RPM (+-half band) / trim");
        for (int i = Math.max(0, spots.size() - 5); i < spots.size(); i++) {
            Spot s = spots.get(i);
            telemetry.addLine(String.format(Locale.US, "  %d: %.0f in %+4.0f -> %s / %s", s.n,
                    s.distance, s.turretDeg,
                    s.rpmBounded ? String.format(Locale.US, "%.0f +-%.0f", s.rpmCentre, s.rpmWidth / 2)
                            : Double.isNaN(s.rpmCentre) ? "--" : String.format(Locale.US, "~%.0f", s.rpmCentre),
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
     * The flywheel-offset fit, once there is enough to fit. RPM model needs 5+
     * bounded spots over 3+ turret angles (it also has a distance term); aim
     * model needs 4+ spots with a trim over 3+ angles.
     */
    private String fitText() {
        List<Spot> r = new ArrayList<>();
        List<Spot> a = new ArrayList<>();
        for (Spot s : spots) {
            if (s.rpmBounded) r.add(s);
            if (!Double.isNaN(s.aimCentre)) a.add(s);
        }
        StringBuilder sb = new StringBuilder("FIT  ");
        double[] c = null;
        if (r.size() >= 5 && angles(r) >= 3) {
            double[][] rows = new double[r.size()][];
            double[] y = new double[r.size()];
            for (int i = 0; i < r.size(); i++) {
                rows[i] = ShotMap.rpmRow(r.get(i).distance, r.get(i).turretDeg);
                y[i] = r.get(i).rpmCentre;
            }
            c = ShotMap.fit(rows, y);
        }
        sb.append(c == null
                ? String.format(Locale.US, "rpm: need 5 bounded spots, 3 angles (%d, %d)", r.size(), angles(r))
                : String.format(Locale.US, "rpm %+.0f cos %+.0f sin (%.1f/in)", c[2], c[3], c[1]));
        double[] k = null;
        if (a.size() >= 4 && angles(a) >= 3) {
            double[][] rows = new double[a.size()][];
            double[] y = new double[a.size()];
            for (int i = 0; i < a.size(); i++) {
                rows[i] = ShotMap.aimRow(a.get(i).turretDeg);
                y[i] = a.get(i).aimCentre;
            }
            k = ShotMap.fit(rows, y);
        }
        sb.append(k == null ? " | aim: need 4 spots, 3 angles"
                : String.format(Locale.US, " | aim %+.1f %+.1fcos %+.1fsin", k[0], k[1], k[2]));
        return sb.toString();
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
