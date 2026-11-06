package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Find the pivot positions, one step at a time. Nothing else.
 *
 * Park the pivot at a hard stop by hand, press PLAY, and the sensor is zeroed
 * there. From then on every reading is degrees from that stop, and every button
 * press moves the pivot exactly one step. Step until the gap looks right, press
 * A for pollen or B for nectar, write the numbers down.
 *
 * NO TRIGGERS, ON PURPOSE. Analog jogging has to multiply by loop time, and
 * once that is wrapped in a rate limiter and a travel clamp it responds
 * unevenly and feels broken. A step is a step: one press, one known change, no
 * timing involved.
 *
 * NO FLYWHEELS, NO INDEXER, NO DEFLECTOR. Shooting Sequence does all of that;
 * this is for tuning the pivot with nothing spinning.
 *
 * THE ONE JUMP THAT CANNOT BE AVOIDED
 * Servo.getPosition() is not a sensor -- it reports the last value commanded,
 * which at op mode start is 0.0 regardless of where the pivot physically is. So
 * the first position this op mode commands is a jump, and software cannot creep
 * into it because it does not know where it is starting from.
 *
 * Which is why PLAY does not move anything. The sensor zeroes, nothing is
 * written to the servo, and the first step press is what engages at
 * START_POSITION. Set that during INIT with dpad up/down so the jump is small:
 * run once, read how far the sensor moved, adjust, run again.
 *
 * Config: "shooterPivot" (Servo). "pivotPos" (Analog Input) is optional but
 * most of the point -- without it there is nothing to zero and you are back to
 * guessing from commanded numbers alone.
 *
 * CONTROLS
 *   INIT    dpad up/down   set the position the first step will command
 *   PLAY                   zeroes the sensor where the pivot is now
 *   dpad up / down         one step tighter / wider  (STEP)
 *   bumpers                one big step tighter / wider  (BIG_STEP)
 *   X                      re-zero the sensor here
 *   A                      record this as POLLEN
 *   B                      record this as NECTAR
 */
@Configurable
@TeleOp(name = "Pivot Finder", group = "Test")
public class PivotFinder extends LinearOpMode {

    /** What the first step press commands. Adjust in INIT; nothing moves before. */
    public static double START_POSITION = 0.30;

    /** One dpad press, in servo position units. */
    public static double STEP = 0.005;

    /** One bumper press. */
    public static double BIG_STEP = 0.02;

    /** Hard limits on what will ever be commanded. Wide: the band is unmeasured. */
    public static double MIN_POSITION = 0.05;
    public static double MAX_POSITION = 0.95;

    /**
     * If a step changes the sensor by less than this, the pivot did not follow --
     * which usually means it is against a hard stop. Shown, not enforced; you
     * are watching the mechanism and the code is not.
     */
    public static double STUCK_VOLTS = 0.004;

    private Servo pivot;
    private AnalogInput pivotPos;

    private double commanded;
    private boolean engaged = false;

    private double zeroVolts = Double.NaN;
    private double voltsBeforeStep = Double.NaN;
    private double lastStepVolts = Double.NaN;

    private double pollen = Double.NaN, pollenDeg = Double.NaN;
    private double nectar = Double.NaN, nectarDeg = Double.NaN;

    private boolean lastUp, lastDown, lastRb, lastLb, lastX, lastA, lastB;

    @Override
    public void runOpMode() {
        pivot = hardwareMap.get(Servo.class, "shooterPivot");
        pivotPos = hardwareMap.tryGet(AnalogInput.class, "pivotPos");

        commanded = START_POSITION;

        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !lastUp) START_POSITION += 0.01;
            if (down && !lastDown) START_POSITION -= 0.01;
            START_POSITION = Range.clip(START_POSITION, MIN_POSITION, MAX_POSITION);
            commanded = START_POSITION;
            lastUp = up; lastDown = down;

            telemetry.addLine("PARK THE PIVOT AT A HARD STOP BY HAND.");
            telemetry.addLine("Nothing is written to the servo yet.");
            telemetry.addLine();
            telemetry.addData("First step will command", "%.3f   <-- dpad up/down",
                    START_POSITION);
            telemetry.addData("pivotPos", pivotPos == null
                    ? "NOT CONFIGURED -- no sensor to zero"
                    : String.format("%.3f V", pivotPos.getVoltage()));
            telemetry.addLine();
            telemetry.addLine("PLAY zeroes the sensor. The pivot still will not move");
            telemetry.addLine("until the first step press.");
            telemetry.update();
        }

        // PLAY: this is the zeroing moment. Nothing moves.
        zeroVolts = pivotPos == null ? Double.NaN : pivotPos.getVoltage();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
            boolean x = gamepad1.x, a = gamepad1.a, b = gamepad1.b;

            if (up && !lastUp) step(+STEP);
            if (down && !lastDown) step(-STEP);
            if (rb && !lastRb) step(+BIG_STEP);
            if (lb && !lastLb) step(-BIG_STEP);

            if (x && !lastX && pivotPos != null) {
                zeroVolts = pivotPos.getVoltage();
            }
            if (a && !lastA && engaged) {
                pollen = commanded;
                pollenDeg = degreesFromZero();
            }
            if (b && !lastB && engaged) {
                nectar = commanded;
                nectarDeg = degreesFromZero();
            }

            lastUp = up; lastDown = down; lastRb = rb; lastLb = lb;
            lastX = x; lastA = a; lastB = b;

            show();
            telemetry.update();
        }
    }

    /**
     * Move one step. The first call engages at START_POSITION instead of
     * stepping, so the unavoidable jump happens on a press you chose to make.
     */
    private void step(double delta) {
        if (pivotPos != null) {
            voltsBeforeStep = pivotPos.getVoltage();
        }

        if (!engaged) {
            engaged = true;
            commanded = Range.clip(START_POSITION, MIN_POSITION, MAX_POSITION);
        } else {
            commanded = Range.clip(commanded + delta, MIN_POSITION, MAX_POSITION);
        }
        pivot.setPosition(commanded);

        // Give the servo a moment, then see whether it actually went anywhere.
        // Blocking is fine here: nothing else is running.
        if (pivotPos != null) {
            sleep(250);
            lastStepVolts = pivotPos.getVoltage() - voltsBeforeStep;
        }
    }

    private double degreesFromZero() {
        if (pivotPos == null || Double.isNaN(zeroVolts)) return Double.NaN;
        double max = pivotPos.getMaxVoltage();
        if (max <= 0) return Double.NaN;
        return (pivotPos.getVoltage() - zeroVolts) / max * 360.0;
    }

    private void show() {
        if (!engaged) {
            telemetry.addLine("SENSOR ZEROED. Nothing has moved yet.");
            telemetry.addLine("Press a step button to engage at " +
                    String.format("%.3f", START_POSITION));
            telemetry.addLine();
        }

        telemetry.addData("Commanded", engaged ? String.format("%.3f", commanded) : "not engaged");
        telemetry.addData("Step size", "%.3f   (bumpers %.3f)", STEP, BIG_STEP);

        if (pivotPos == null) {
            telemetry.addLine();
            telemetry.addLine("pivotPos NOT CONFIGURED -- commanded position is all there is.");
        } else {
            telemetry.addLine();
            telemetry.addData("Sensor", "%.3f V", pivotPos.getVoltage());
            telemetry.addData("From the stop", "%+.3f V   =  %+.1f deg",
                    pivotPos.getVoltage() - zeroVolts, degreesFromZero());
            if (!Double.isNaN(lastStepVolts)) {
                telemetry.addData("Last step moved", "%+.4f V%s", lastStepVolts,
                        Math.abs(lastStepVolts) < STUCK_VOLTS
                                ? "   <-- DID NOT MOVE. Hard stop?" : "");
            }
        }

        if (!Double.isNaN(pollen) || !Double.isNaN(nectar)) {
            telemetry.addLine();
            if (!Double.isNaN(pollen)) {
                telemetry.addData("POLLEN", "%.3f   (%+.1f deg from stop)", pollen, pollenDeg);
            }
            if (!Double.isNaN(nectar)) {
                telemetry.addData("NECTAR", "%.3f   (%+.1f deg from stop)", nectar, nectarDeg);
            }
        }

        telemetry.addLine();
        telemetry.addLine("dpad = step   bumpers = big step   X = re-zero");
        telemetry.addLine("A = record pollen   B = record nectar");
    }
}
