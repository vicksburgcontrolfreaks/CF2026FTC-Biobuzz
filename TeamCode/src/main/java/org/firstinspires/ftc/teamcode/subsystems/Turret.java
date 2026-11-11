package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.WhichRobot;

/**
 * Turret subsystem: a continuous-rotation servo turning a 30T drive gear into a
 * 69T turret gear (2.3:1 reduction), with a REV Through Bore encoder for position
 * and a magnetic limit switch for homing.
 *
 * A CR servo takes a POWER, not a position -- it has no idea where it is. So this
 * class closes the loop itself: read the encoder, compare to the target angle,
 * and pick a power. That also means the turret does not know where it is pointed
 * until it has been homed, which is why there is a state machine here instead of
 * just a setAngle() method.
 *
 * Driver Station config names this expects:
 *   "turret"     - CRServo (Continuous Rotation Servo) on a servo port
 *   "turretHome" - REV Touch Sensor (the magnetic limit switch), digital port
 *   "lf"         - the LEFT FRONT DRIVE MOTOR. The turret encoder is plugged into
 *                  that motor port's spare encoder connector, so there is no
 *                  separate encoder device to configure. Read the warning below.
 *
 * SHARING THE "lf" MOTOR PORT
 * The drivetrain owns "lf". This class only ever READS its encoder -- it never
 * sets power, direction, run mode, or resets the encoder on it. Two rules keep
 * that safe, and breaking either one breaks the turret:
 *
 *   1. "lf" must stay in RUN_WITHOUT_ENCODER (the default). If anything puts it
 *      in RUN_USING_ENCODER or RUN_TO_POSITION, the hub's velocity PID would try
 *      to regulate the drive wheel using the TURRET's position. Nothing in this
 *      repo does that today, and Pedro Pathing's Mecanum class does not either.
 *   2. Nobody calls STOP_AND_RESET_ENCODER on "lf", which would zero the turret's
 *      position mid-match.
 *
 * Direction is handled for us: the SDK negates getCurrentPosition() when a motor
 * is REVERSE (DcMotorImpl.adjustPosition), and lf's direction belongs to the
 * drivetrain -- it has changed before (Pedro's Constants vs RobotHardware) and
 * may change again. readRawTicks() divides that out so turret angles
 * mean the same thing everywhere, and ENCODER_DIRECTION stays the only sign knob.
 *
 * Call init() once, then update() every loop. Nothing moves unless update() runs.
 */
@Configurable
public class Turret {

    // ---- Geometry -----------------------------------------------------------
    // Ratio check: the servo must turn 69/30 = 2.3 revs for one full turret rev.
    public static double DRIVE_GEAR_TEETH = 20.0;
    public static double TURRET_GEAR_TEETH = 69.0;

    // DERIVED FROM GEAR TEETH, not from a sweep -- this one is exact:
    //   REV Through Bore V2 (REV-11-3174)        = 8192 counts/rev
    //   encoder gear 20T on the 69T ring         = 3.45 : 1
    //   8192 * 3.45 / 360                        = 78.51 ticks/deg
    //
    // Every term is an integer from a spec sheet or a tooth count, so there is no
    // measurement error in this number at all. Confirmed in CAD 2026-10-15: both
    // small gears are OD 44 mm and the ring is OD 142 mm, which at module 2 is
    // 20T (44 = 2 * (20+2)) and 69T (142 = 2 * (69+2)).
    //
    // The encoder does NOT ride the servo's pinion -- it has its own 20T gear
    // meshing the same ring, so it happens to share the 3.45:1 ratio.
    //
    // MEASURED 2026-10-13 with TurretRangeTest, pushing by hand to both hard stops:
    //   stop A -9617 ticks, magnet -124, stop B +12832  ->  span 22449 ticks
    // Against 78.51 ticks/deg that span is 286 deg of travel. Two earlier values
    // were wrong: 106.90 (span / a guessed 210 deg) and 95.73 (span / a CAD angle
    // read as 234.5, later 232). A CAD angle of 232 deg would require the span to
    // be 18214 ticks; the encoder counted 22449. Encoders do not miscount by 23%,
    // and a tape-and-count sweep gave ~2.6 encoder turns against the 2.74 that
    // 22449/8192 implies -- the 232 deg reading predicts only 2.22 turns.
    public static double TICKS_PER_DEGREE = 78.51;

    // Set to -1 if positive servo power makes the reported angle go DOWN.
    // You will find this out in the first 10 seconds of testing. This is the ONLY
    // place turret sign should be adjusted -- do not "fix" it by changing lf's
    // direction, which belongs to the drivetrain.
    public static double ENCODER_DIRECTION = 1.0;

    // The drive motor whose spare encoder port the turret encoder is wired into.
    private static final String ENCODER_MOTOR_NAME = "lf";

    // ---- Travel limits ------------------------------------------------------
    // Travel works out to 286 degrees, so the hard stops sit at -143.0 and +143.0
    // either side of center. These limits are pulled in 3 degrees from the stops so
    // the turret never rests against one; widen toward 143 if the extra reach is
    // needed, but do not exceed it.
    //
    // Because ticks-per-degree and these limits both trace back to the same 22449
    // tick sweep, the limits land on the real stops even if the degree scale is
    // later revised -- a scale error shifts the labels, not where the turret stops.
    //
    // 6494, 2026-11-06 (Shane): the turret NO LONGER HAS HARD STOPS -- the cable
    // wrap is the real limit. Shane swept by hand to where it should stop: +/-109.4
    // either side of the sweep's middle, so +/-106 with the same 3 deg margin.
    // Everything above about 286 deg / stops at +/-143 is the OLD build.
    //
    // PER ROBOT: MIN_ANGLE, MAX_ANGLE, HOME_SWITCH_ANGLE, HOMING_DIRECTION and
    // HOMING_SEARCH_MAX_DEG are overwritten by init() from loadRobotNumbers(). Edit
    // the numbers THERE. The defaults written here are 6494's, only so laptop code
    // that reads them before any init() sees sane values.
    public static double MIN_ANGLE = -90.0;
    public static double MAX_ANGLE = 90.0;

    // ---- Homing -------------------------------------------------------------
    // MEASURED 2026-10-15 on the rebuilt magnet mount, three Turret Range Test
    // sweeps: -21.8, -21.3, -22.1 -> mean -21.73, spread 0.8 deg. The magnet sits
    // near the MIDDLE of travel, about 121 degrees from one hard stop and 165 from
    // the other, which is why homing searches both directions.
    //
    // THIS IS AN EDGE, NOT THE BAND CENTER. The band is wide -- 12.5 deg, and it
    // wandered from 11.4 to 13.6 across the three sweeps -- so its center would be a
    // poor reference. Homing instead always makes its final approach travelling in
    // HOMING_DIRECTION, and this is the angle where the switch first trips coming
    // that way. Re-measure it the same way after any change to the magnet or mount.
    //
    // The 0.8 deg spread is hand-sweep noise: how fast you push changes where the
    // trip gets sampled, since the switch is only read once per loop. The powered
    // approach runs at a fixed HOMING_APPROACH_POWER, so it should repeat better.
    // Once homing works, home twice and compare the resting angle to find out.
    // RE-MEASURED 6494, 2026-11-06 (Shane, one Turret Range Test sweep): -80.5, band
    // 10.1 deg (-90.6 to -80.5). The magnet moved ~59 deg in the 2026 rebuild, which
    // is why homing parked ~65 deg off forward. The 10-15 history above is the old mount.
    // -80.5 parked ~5 deg CCW of true forward (Shane, by eye), so +5: -75.5.
    // -75.5 then parked 10 deg CW of forward (Shane jogged it, read the angle): -85.5.
    // Per robot -- see loadRobotNumbers().
    public static double HOME_SWITCH_ANGLE = -85.5;

    // Width of the magnet's trip band, measured from HOME_SWITCH_ANGLE going in
    // HOMING_DIRECTION. Only used to sanity-check a kept homing: if the switch
    // trips somewhere the band can't be, the zero is wrong. Per robot.
    public static double MAGNET_BAND_DEG = 10.1;
    /** Slack on that check, for loop timing when the turret passes the band fast. */
    public static double MAGNET_CHECK_SLACK_DEG = 5.0;

    // Which way to search FIRST, and -- more importantly -- the direction the final
    // approach always comes from. Clockwise (-): the side with cable slack, and the
    // magnet is near that end. See HOMING_SEARCH_MAX_DEG. Per robot.
    public static double HOMING_DIRECTION = -1.0;
    public static double HOMING_POWER = 0.25;

    // ---- Homing: how far to search (6494, 2026-11-06) ------------------------
    // There are no hard stops any more, so "search until you stall, then reverse"
    // no longer works -- nothing stops it. The search is limited by DISTANCE instead,
    // measured on the encoder (which counts fine before homing; only the zero is
    // unknown).
    //
    // ONE direction, a fixed distance, then FAIL -- no reversing. Without an
    // absolute sensor the turret can't know which side of the magnet it's on, and
    // a turret that started past the magnet would otherwise run the long way
    // round: the first try (search ~206 deg, then reverse) did exactly that and had
    // to be stopped by hand (6494, 2026-11-06).
    //
    // So the rule is: the magnet is never behind you when homing starts.
    //   - MIN_ANGLE sits INSIDE the magnet band, so a homed turret can't park
    //     past it.
    //   - Before homing, the drive team sets it roughly forward (by hand, robot
    //     off). From roughly forward the magnet is well inside this distance.
    //   - If it's not found anyway: FAILED, "turn it forward by hand, press X".
    //     Worst case, it went this far the wrong way, into the cable's slack side.
    // Per robot -- see loadRobotNumbers().
    public static double HOMING_SEARCH_MAX_DEG = 150.0;

    // Backstop only: if a leg takes longer than this, the servo is too slow or not
    // moving, so FAIL (don't reverse). Distance, not time, decides the reversal --
    // at ~27 deg/s, 186 deg is ~7 s, which the old 6 s timer could cut short and
    // then send the turret CCW into the tight cable.
    public static double HOMING_TIMEOUT_SEC = 15.0;

    // ---- Homing: the final approach -----------------------------------------
    // The magnet trigger band is WIDE -- measured at 942 ticks, about 12 degrees.
    // Where the switch first trips therefore depends on which side you came at it
    // from, and the coarse search can arrive from either side. Taking that first
    // trip as the zero reference would put the turret whole angle frame out by up
    // to a full band width.
    //
    // So homing never uses the coarse trip. Once the search finds the magnet it
    // backs off clear of the band and re-approaches in HOMING_DIRECTION, every time,
    // at a lower power. The reference is then always the same physical edge, reached
    // the same way, and repeatability is set by loop timing rather than by 12 degrees
    // of band.
    //
    // THIS CHANGES WHAT HOME_SWITCH_ANGLE MEANS: it is the angle of that one edge --
    // the first trip when travelling in HOMING_DIRECTION -- NOT the center of the
    // band. Re-measure it that way after any change to the magnet or its mount.
    public static double HOMING_BACKOFF_DEG = 8.0;

    // Must stay above MIN_POWER or the turret will not move at all. Slower is more
    // repeatable, because the trip is only noticed once per loop and whatever the
    // turret travels during that loop is error. Too slow and the stall detector may
    // fire on the approach -- if that happens, lower STALL_TICKS before raising this.
    public static double HOMING_APPROACH_POWER = 0.15;

    // After homing, drive here so the turret is not resting against the stop.
    public static double PARK_ANGLE = 0.0;

    // ---- Closed loop --------------------------------------------------------
    // Per robot -- kP and MAX_POWER are overwritten by loadRobotNumbers().
    // MAX_POWER also caps JOG (hand turret on gamepad2), so raising it speeds that up too.
    public static double kP = 0.010;          // power per degree of error
    public static double MAX_POWER = 0.55;
    public static double MIN_POWER = 0.12;    // deadband: below this the servo just buzzes
    public static double TOLERANCE_DEG = 2.0; // close enough -- stop pushing

    // ---- Stall detection ----------------------------------------------------
    // If we are commanding power but the encoder is not moving, we have hit
    // something. Give up instead of grinding the gears.
    public static double STALL_TICKS = 15.0;
    public static double STALL_TIME_SEC = 0.35;

    /** NOT_HOMED: angle unknown, soft limits off. READY: angle known, limits on. */
    public enum State { NOT_HOMED, HOMING, READY, FAILED }

    /** HOLD runs the closed loop to targetAngle. JOG passes raw power through. */
    public enum ControlMode { HOLD, JOG }

    private CRServo servo;
    /** Shared with the drivetrain. READ ONLY -- see the class comment. */
    private DcMotorEx encoderMotor;
    private TouchSensor homeSwitch;

    private State state = State.NOT_HOMED;
    private ControlMode controlMode = ControlMode.JOG;

    private double homeTickOffset = 0.0;   // encoder reading taken at referenceAngle
    private double referenceAngle = 0.0;   // the angle that offset corresponds to
    private double targetAngle = 0.0;
    private double rawPower = 0.0;         // what JOG mode sends
    private double lastAppliedPower = 0.0;
    private String statusNote = "init";

    private final ElapsedTime homingTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private int lastStallTicks = 0;

    /** Homing runs SEARCH -> BACKOFF -> APPROACH; only APPROACH sets the reference. */
    private enum HomingPhase { SEARCH, BACKOFF, APPROACH }

    // Homing search state. One direction only (see HOMING_SEARCH_MAX_DEG).
    private double homingSearchDir = HOMING_DIRECTION;
    private int searchLegStartTicks = 0;       // reading when this search leg began
    private HomingPhase homingPhase = HomingPhase.SEARCH;
    private boolean backoffReleased = false;   // has the switch let go yet?
    private int backoffStartTicks = 0;         // reading when it let go

    // ---- Homing kept between op modes (Shane, 2026-11-06) -------------------
    // Home ONCE per power-up, not once per op mode: auto homes, TeleOp reuses it.
    // Saves match time, and means homing happens when the robot is being set up
    // by hand -- the easy moment to follow "start the turret roughly forward".
    //
    // Why this is safe:
    //   - The hub keeps counting the encoder between op modes; nothing in this
    //     repo resets lf's encoder (see the class comment, rule 2).
    //   - STATIC fields live as long as the Robot Controller app. A power cycle or
    //     a redeploy restarts the app, wipes these, and homing is required again --
    //     exactly when the encoder count has also been lost.
    //   - Belt and braces: while READY, if the magnet trips where its band can't
    //     be (checkMagnetAgreesWithAngle), the zero is wrong -> back to NOT_HOMED.
    // X always re-homes from scratch, kept homing or not.
    private static boolean keptHomed = false;
    private static double keptTickOffset = 0.0;
    private static double keptReferenceAngle = 0.0;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, hardwareMap.get(DcMotorEx.class, ENCODER_MOTOR_NAME));
    }

    /**
     * Same as init(hardwareMap) but reuses a drive motor handle the caller already
     * has. Use this from RobotHardware so there is one lf object, not two.
     */
    public void init(HardwareMap hardwareMap, DcMotorEx sharedEncoderMotor) {
        servo = hardwareMap.get(CRServo.class, "turret");
        encoderMotor = sharedEncoderMotor;
        homeSwitch = hardwareMap.get(TouchSensor.class, "turretHome");

        servo.setPower(0.0);
        loadRobotNumbers();

        // Treat wherever we booted up as angle 0 until homing proves otherwise.
        // Soft limits stay OFF while NOT_HOMED, because this number is a guess.
        homeTickOffset = readRawTicks();
        referenceAngle = 0.0;
        state = State.NOT_HOMED;
        statusNote = "not homed";

        if (keptHomed && isMeasured()) {
            homeTickOffset = keptTickOffset;
            referenceAngle = keptReferenceAngle;
            state = State.READY;
            controlMode = ControlMode.HOLD;
            targetAngle = getAngle();   // hold where it is; don't lurch on init
            statusNote = "homed (kept from an earlier op mode)";
        }
    }

    /** True if init() reused an earlier op mode's homing instead of starting NOT_HOMED. */
    public boolean wasHomingKept() {
        return keptHomed && state == State.READY;
    }

    private static void forgetHoming() {
        keptHomed = false;
    }

    /**
     * While READY, a tripped switch must be inside the magnet's band. If it isn't,
     * the zero is wrong (encoder reset, belt slipped, ...): drop to NOT_HOMED.
     */
    private void checkMagnetAgreesWithAngle() {
        if (state != State.READY || Double.isNaN(MAGNET_BAND_DEG) || !homeSwitch.isPressed()) {
            return;
        }
        double bandFar = HOME_SWITCH_ANGLE + HOMING_DIRECTION * MAGNET_BAND_DEG;
        double lo = Math.min(HOME_SWITCH_ANGLE, bandFar) - MAGNET_CHECK_SLACK_DEG;
        double hi = Math.max(HOME_SWITCH_ANGLE, bandFar) + MAGNET_CHECK_SLACK_DEG;
        double angle = getAngle();
        if (angle < lo || angle > hi) {
            forgetHoming();
            state = State.NOT_HOMED;
            controlMode = ControlMode.JOG;
            statusNote = String.format("ZERO LOST - magnet seen at %.0f deg, expected %.0f..%.0f."
                    + " Press X to re-home.", angle, lo, hi);
        }
    }

    // ---- Commands -----------------------------------------------------------

    /**
     * The numbers that differ between the two robots' turrets: where the magnet
     * is, where the cable says stop, and which way has the slack. Same idea as
     * pedroPathing/Constants for the drivetrain: one place, picked by the config
     * name, so deploying to the other robot can't run this one's numbers.
     *
     * NaN HOME_SWITCH_ANGLE = "not measured": jogging works, homing refuses. That
     * stops a guessed magnet position from parking the turret wrong or homing it
     * toward the tight side of its cable.
     */
    private static void loadRobotNumbers() {
        switch (WhichRobot.get()) {
            case R5618:
                // Turret Range Test, Shane, 2026-11-06: cable stops +-93.3 (travel
                // 186.6), magnet band -91.0..-83.1 (8 deg), truly forward read 0.0
                // in the same frame, so no forward correction. Same layout as 6494:
                // angle falls clockwise, cable slack clockwise, tight counter-clockwise.
                HOME_SWITCH_ANGLE = -83.1;
                MAGNET_BAND_DEG = 7.9;
                // -90: inside the magnet band (same rule as 6494), 3.3 off the stop.
                MIN_ANGLE = -90.0;
                // +85, not +90: +93.3 is the TIGHT cable end, so +90 would leave only
                // 3.3 deg for overshoot. 8.3 deg of margin instead.
                MAX_ANGLE = 85.0;
                HOMING_DIRECTION = -1.0;   // toward the slack (clockwise)
                // Finds the magnet from any start between it and +66.9 (-83.1 + 150).
                HOMING_SEARCH_MAX_DEG = 150.0;
                // Original speed, unchanged.
                kP = 0.010;
                MAX_POWER = 0.55;
                break;
            case R6494:
            default:
                // Shane, 2026-11-06. No hard stops; cable has slack clockwise (-),
                // tight counter-clockwise (+). Magnet band -90.6..-80.5 on the
                // Range Test sweep; -85.5 after two by-eye forward corrections, so the
                // band is about -95.6..-85.5 in this frame. Homing repeatable (Shane).
                HOME_SWITCH_ANGLE = -85.5;
                MAGNET_BAND_DEG = 10.1;   // Range Test sweep
                // -90, not -106: INSIDE the magnet band, so a homed turret can never
                // park on the far side of the magnet, where homing can't find it
                // (long-way test failed, 2026-11-06). Costs 16 deg of clockwise reach.
                MIN_ANGLE = -90.0;
                // +90, not +106: symmetric with -90, and keeps well off the tight
                // side of the cable (Shane, 2026-11-06).
                MAX_ANGLE = 90.0;
                HOMING_DIRECTION = -1.0;   // toward the slack
                // Finds the magnet from any start between it and +64.5 (-85.5 + 150),
                // i.e. anywhere roughly forward. Beyond that: FAILED, turn it by hand.
                HOMING_SEARCH_MAX_DEG = 150.0;
                // 2026-11-10, NOT YET ROBOT-TESTED: faster turret trial (was kP 0.010,
                // MAX_POWER 0.55). kP doubled so small corrections push harder; cap at
                // 0.8, not 1.0, because 6494 has NO hard stops and a faster turret
                // coasts further past the +90 soft limit toward the tight cable end.
                // If it oscillates around the target, lower kP before touching MAX_POWER.
                kP = 0.020;
                MAX_POWER = 0.80;
                break;
        }
    }

    /** True once this robot's magnet position has been measured (see loadRobotNumbers). */
    public static boolean isMeasured() {
        return !Double.isNaN(HOME_SWITCH_ANGLE);
    }

    /** Begin homing. It finishes inside update() over the next few seconds. */
    public void startHoming() {
        if (!isMeasured()) {
            state = State.NOT_HOMED;
            statusNote = "robot " + WhichRobot.number()
                    + " turret NOT MEASURED - run Turret Range Test (see Turret.loadRobotNumbers)";
            return;
        }
        forgetHoming();   // re-homing from scratch; kept again only if it succeeds
        state = State.HOMING;
        controlMode = ControlMode.HOLD;
        homingSearchDir = HOMING_DIRECTION;
        searchLegStartTicks = readRawTicks();
        homingPhase = HomingPhase.SEARCH;
        backoffReleased = false;
        homingTimer.reset();
        resetStallWatch();
        statusNote = "homing: searching " + (homingSearchDir < 0 ? "-" : "+");
    }

    /** Closed-loop target in turret degrees. Clamped to the soft limits once homed. */
    public void setTargetAngle(double degrees) {
        targetAngle = isHomed() ? Range.clip(degrees, MIN_ANGLE, MAX_ANGLE) : degrees;
    }

    /** Raw power for JOG mode, -1..1. Soft limits still apply once homed. */
    public void setRawPower(double power) {
        rawPower = Range.clip(power, -1.0, 1.0);
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
        if (mode == ControlMode.HOLD) {
            targetAngle = getAngle();   // do not lurch: hold where we are right now
        }
    }

    /**
     * Declare "the turret is physically at this angle right now" without homing.
     * This is for measuring TICKS_PER_DEGREE during testing, not for match code.
     *
     * Two things it must also do (both bit on 6494, 2026-11-06):
     *   - Move the HOLD target with the reference. Otherwise the old target number
     *     is now measured from the new zero, and the turret lurches to it.
     *   - Drop out of READY. The soft limits are measured from the HOMED zero; from
     *     a hand-set zero they no longer line up with the hard stops and can drive
     *     the turret straight into one. Back to NOT_HOMED (jog only, no limits)
     *     until X re-homes.
     */
    public void resetReferenceHere(double assumedAngle) {
        homeTickOffset = readRawTicks();
        referenceAngle = assumedAngle;
        targetAngle = assumedAngle;
        forgetHoming();   // a hand-set zero must never carry into the next op mode
        if (state == State.READY) {
            state = State.NOT_HOMED;
            controlMode = ControlMode.JOG;
            statusNote = "zero set by hand (B) - soft limits OFF, X to re-home";
        }
    }

    /** Run every loop. This is the only place the servo is actually written to. */
    public void update() {
        double power;
        checkMagnetAgreesWithAngle();

        switch (state) {
            case HOMING:
                power = updateHoming();
                break;

            case READY:
                power = (controlMode == ControlMode.JOG) ? rawPower : closedLoopPower();
                power = applySoftLimits(power);
                break;

            case NOT_HOMED:
                // Jogging is allowed so you can find the magnet and measure the
                // encoder, but there are no soft limits to protect you. Go slow.
                power = (controlMode == ControlMode.JOG) ? rawPower : 0.0;
                break;

            case FAILED:
            default:
                power = 0.0;
                break;
        }

        lastAppliedPower = Range.clip(power, -MAX_POWER, MAX_POWER);
        servo.setPower(lastAppliedPower);
    }

    /** Cut power immediately and hold wherever we ended up. */
    public void stop() {
        rawPower = 0.0;
        lastAppliedPower = 0.0;
        if (servo != null) {
            servo.setPower(0.0);
        }
    }

    // ---- Internals ----------------------------------------------------------

    private double updateHoming() {
        switch (homingPhase) {
            case SEARCH:   return homingSearch();
            case BACKOFF:  return homingBackoff();
            case APPROACH: return homingApproach();
            default:       return 0.0;
        }
    }

    /** Coarse hunt for the magnet, either direction. Does NOT set the reference. */
    private double homingSearch() {
        if (homeSwitch.isPressed()) {
            // Found it -- but which edge this is depends on which way we arrived,
            // and the band is 12 degrees wide. Back off and come in the standard way.
            homingPhase = HomingPhase.BACKOFF;
            backoffReleased = false;
            homingTimer.reset();
            resetStallWatch();
            statusNote = "found magnet, backing off";
            return 0.0;
        }

        // See HOMING_SEARCH_MAX_DEG: one direction, a fixed distance, then give up.
        double legDeg = Math.abs(readRawTicks() - searchLegStartTicks) / TICKS_PER_DEGREE;

        if (homingTimer.seconds() > HOMING_TIMEOUT_SEC) {
            state = State.FAILED;
            statusNote = String.format(
                    "HOMING FAILED - only %.0f deg in %.0f s; servo slow or not moving?",
                    legDeg, HOMING_TIMEOUT_SEC);
            return 0.0;
        }
        if (isStalled()) {
            state = State.FAILED;
            statusNote = "HOMING FAILED - stalled; something blocking the turret?";
            return 0.0;
        }
        if (legDeg >= HOMING_SEARCH_MAX_DEG) {
            state = State.FAILED;
            statusNote = String.format("HOMING FAILED - no magnet in %.0f deg. Was it past "
                    + "the magnet? Turn it to face FORWARD by hand, then X.", legDeg);
            return 0.0;
        }

        return homingSearchDir * HOMING_POWER;
    }

    /**
     * Retreat against HOMING_DIRECTION until the switch releases, then
     * HOMING_BACKOFF_DEG further, so the re-approach always starts outside the band.
     * Waiting for the release first means this works whatever the band width is.
     */
    private double homingBackoff() {
        if (!backoffReleased) {
            if (!homeSwitch.isPressed()) {
                backoffReleased = true;
                backoffStartTicks = readRawTicks();
            }
        } else if (Math.abs(readRawTicks() - backoffStartTicks)
                >= HOMING_BACKOFF_DEG * TICKS_PER_DEGREE) {
            homingPhase = HomingPhase.APPROACH;
            homingTimer.reset();
            resetStallWatch();
            statusNote = "re-approaching slowly";
            return 0.0;
        }

        if (homingTimer.seconds() > HOMING_TIMEOUT_SEC || isStalled()) {
            // Only plausible if the magnet sat right against a hard stop, which it
            // does not -- so this almost certainly means the switch is stuck on.
            state = State.FAILED;
            statusNote = "HOMING FAILED - could not back off the magnet; switch stuck?";
            return 0.0;
        }

        return -HOMING_DIRECTION * HOMING_POWER;
    }

    /** The only place the zero reference is ever set. Always the same edge. */
    private double homingApproach() {
        if (homeSwitch.isPressed()) {
            homeTickOffset = readRawTicks();
            referenceAngle = HOME_SWITCH_ANGLE;
            state = State.READY;
            controlMode = ControlMode.HOLD;
            targetAngle = Range.clip(PARK_ANGLE, MIN_ANGLE, MAX_ANGLE);
            statusNote = "homed, parking";
            keptTickOffset = homeTickOffset;     // later op modes reuse this
            keptReferenceAngle = referenceAngle;
            keptHomed = true;
            return 0.0;
        }

        if (homingTimer.seconds() > HOMING_TIMEOUT_SEC || isStalled()) {
            state = State.FAILED;
            statusNote = "HOMING FAILED - magnet not seen on re-approach; "
                    + "is HOMING_APPROACH_POWER above MIN_POWER?";
            return 0.0;
        }

        return HOMING_DIRECTION * HOMING_APPROACH_POWER;
    }

    private double closedLoopPower() {
        double error = targetAngle - getAngle();
        if (Math.abs(error) <= TOLERANCE_DEG) {
            statusNote = "on target";
            return 0.0;
        }
        statusNote = "seeking";

        double power = kP * error;
        // Deadband compensation: below roughly 0.1 power a CR servo just hums,
        // burns current, and never actually turns the turret.
        if (Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }
        return power;
    }

    /** Refuse to drive further past a limit, but always allow coming back. */
    private double applySoftLimits(double power) {
        double angle = getAngle();
        if (angle >= MAX_ANGLE && power > 0.0) {
            statusNote = "at MAX limit";
            return 0.0;
        }
        if (angle <= MIN_ANGLE && power < 0.0) {
            statusNote = "at MIN limit";
            return 0.0;
        }
        return power;
    }

    /**
     * Encoder ticks with the drive motor's direction setting cancelled out.
     *
     * The SDK runs getCurrentPosition() through DcMotorImpl.adjustPosition(), which
     * negates the value when the motor is REVERSE. lf's direction is the
     * drivetrain's business and is not consistent across op modes, so undoing it
     * here keeps a given turret angle meaning the same thing in TeleOp and in auto.
     * Re-read every call rather than cached at init, in case something changes
     * lf's direction while the op mode is running.
     */
    private int readRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        if (encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            ticks = -ticks;
        }
        return ticks;
    }

    private void resetStallWatch() {
        stallTimer.reset();
        lastStallTicks = readRawTicks();
    }

    private boolean isStalled() {
        int now = readRawTicks();
        if (Math.abs(now - lastStallTicks) > STALL_TICKS) {
            resetStallWatch();
            return false;
        }
        return stallTimer.seconds() > STALL_TIME_SEC;
    }

    // ---- Readouts (for telemetry) -------------------------------------------

    /** Turret angle in degrees. Only meaningful once isHomed() is true. */
    public double getAngle() {
        double ticks = (readRawTicks() - homeTickOffset) * ENCODER_DIRECTION;
        return referenceAngle + ticks / TICKS_PER_DEGREE;
    }

    public int getRawTicks() {
        return readRawTicks();
    }

    public int getTicksSinceReference() {
        return (int) (readRawTicks() - homeTickOffset);
    }

    public boolean isHomeSwitchPressed() {
        return homeSwitch.isPressed();
    }

    public boolean isHomed() {
        return state == State.READY;
    }

    public boolean isOnTarget() {
        return isHomed() && Math.abs(targetAngle - getAngle()) <= TOLERANCE_DEG;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getAppliedPower() {
        return lastAppliedPower;
    }

    public State getState() {
        return state;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public String getStatusNote() {
        return statusNote;
    }

    /** Servo revs per turret rev -- a sanity check on the gearing numbers. */
    public static double gearRatio() {
        return TURRET_GEAR_TEETH / DRIVE_GEAR_TEETH;
    }
}
