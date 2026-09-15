package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Cycles the REV Blinkin (servo port 1, configured as "blinkin") through
 * red, white, and blue for decoration. No gamepad input needed - just
 * init and run to watch it cycle.
 */
@TeleOp(name = "LED Test - Red White Blue", group = "Test")
public class LedRedWhiteBlueTest extends OpMode {

    private static final double SECONDS_PER_COLOR = 1.0;

    private final RevBlinkinLedDriver.BlinkinPattern[] colors = {
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.WHITE,
            RevBlinkinLedDriver.BlinkinPattern.BLUE
    };

    private RevBlinkinLedDriver blinkin;
    private int colorIndex = 0;
    private double lastSwitchTime = 0;

    @Override
    public void init() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(colors[colorIndex]);
        telemetry.addData("Color", colors[colorIndex].toString());
    }

    @Override
    public void loop() {
        if (getRuntime() - lastSwitchTime >= SECONDS_PER_COLOR) {
            colorIndex = (colorIndex + 1) % colors.length;
            blinkin.setPattern(colors[colorIndex]);
            lastSwitchTime = getRuntime();
        }
        telemetry.addData("Color", colors[colorIndex].toString());
        telemetry.update();
    }
}
