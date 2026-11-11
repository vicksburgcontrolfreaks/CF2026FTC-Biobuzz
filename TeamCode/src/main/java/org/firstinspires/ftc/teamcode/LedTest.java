package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Leds;

/**
 * Browse the Blinkin's patterns and find the ones worth using.
 *
 * BlinkinPattern has around a hundred values with names like
 * LIGHT_CHASE_GRAY and TWINKLES_PARTY_PALETTE, and no name tells you what it
 * actually looks like on a strip. So step through them, watch the robot, and
 * write down the ones you want.
 *
 * Emmett's "LED Test - Red White Blue" is the decorative fixed cycle. This is
 * the one for picking patterns in the first place, and for checking the strip is
 * wired and configured at all.
 *
 * WHAT GOES IN THE CODE AFTERWARDS
 * The names you settle on go into Leds (IDLE / READY / BUSY / LOADED /
 * NOT_READY), not into op modes -- so op modes ask for a meaning and this stays
 * the only place that knows which pattern it maps to.
 *
 * CONTROLS
 *   dpad up / down     next / previous pattern
 *   bumpers            jump 10 at a time
 *   A                  jump to the pattern Leds.IDLE uses
 *   B                  jump to Leds.READY
 *   X                  BLACK (lights out)
 *   Y                  hold to auto-advance, about 2 per second
 */
@TeleOp(name = "LED Pattern Browser", group = "Test")
public class LedTest extends LinearOpMode {

    private static final BlinkinPattern[] ALL = BlinkinPattern.values();
    private static final double AUTO_SECONDS = 0.5;

    private int index = 0;
    private double lastAuto = 0;
    private boolean lastUp, lastDown, lastRb, lastLb, lastA, lastB, lastX;

    @Override
    public void runOpMode() {
        Leds leds = new Leds(hardwareMap);

        while (opModeInInit()) {
            telemetry.addData("Blinkin", leds.isPresent() ? "found" : "NOT FOUND");
            if (!leds.isPresent()) {
                telemetry.addLine();
                telemetry.addLine("No \"blinkin\" device. It has to be configured on a");
                telemetry.addLine("servo port as type \"REV Blinkin LED Driver\" --");
                telemetry.addLine("NOT as \"Servo\". Those are different config entries");
                telemetry.addLine("and only the first can be read as a Blinkin.");
            }
            telemetry.addLine();
            telemetry.addData("Patterns available", ALL.length);
            telemetry.addData("Writes to hardware", leds.getWriteCount());
            // LIGHT UP NOW, IN INIT. An LED test whose strip stays dark until
            // START cannot prove anything at the moment it says "found" --
            // which is exactly when someone is stood there looking at the robot
            // wondering why nothing happened.
            leds.set(Leds.IDLE);
            telemetry.addLine("START, then dpad up/down to step through them.");
            telemetry.update();
        }

        index = indexOf(Leds.IDLE);
        leds.set(ALL[index]);

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x;

            if (up && !lastUp) index++;
            if (down && !lastDown) index--;
            if (rb && !lastRb) index += 10;
            if (lb && !lastLb) index -= 10;
            if (a && !lastA) index = indexOf(Leds.IDLE);
            if (b && !lastB) index = indexOf(Leds.READY);
            if (x && !lastX) index = indexOf(BlinkinPattern.BLACK);

            if (gamepad1.y && getRuntime() - lastAuto >= AUTO_SECONDS) {
                index++;
                lastAuto = getRuntime();
            }

            // Wrap, so stepping off either end carries on rather than sticking.
            index = ((index % ALL.length) + ALL.length) % ALL.length;
            leds.set(ALL[index]);

            lastUp = up; lastDown = down; lastRb = rb; lastLb = lb;
            lastA = a; lastB = b; lastX = x;

            telemetry.addData("Pattern", "%d of %d", index + 1, ALL.length);
            telemetry.addData("Name", ALL[index].toString());
            telemetry.addData("Writes to hardware", "%d   (re-sent every %.2f s)",
                    leds.getWriteCount(), Leds.REASSERT_SECONDS);
            telemetry.addLine();
            telemetry.addLine("Write the NAME down -- that is what goes into Leds.");
            telemetry.addLine();
            telemetry.addLine("dpad = step   bumpers = jump 10   Y held = auto-advance");
            telemetry.addLine("A = Leds.IDLE   B = Leds.READY   X = lights out");
            if (!leds.isPresent()) {
                telemetry.addLine();
                telemetry.addLine("(No blinkin found -- nothing is lighting up.)");
            }
            telemetry.update();
        }

        leds.off();
    }

    private static int indexOf(BlinkinPattern pattern) {
        for (int i = 0; i < ALL.length; i++) {
            if (ALL[i] == pattern) return i;
        }
        return 0;
    }
}