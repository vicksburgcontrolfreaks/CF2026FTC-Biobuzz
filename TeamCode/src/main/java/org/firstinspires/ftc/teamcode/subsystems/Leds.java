package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The REV Blinkin LED strip. One place to say what the robot should look like,
 * so op modes ask for a meaning rather than a colour.
 *
 * Config: "blinkin" on a servo port (CH servo 1), device type "REV Blinkin LED
 * Driver" -- NOT "Servo". Those are two different config entries and only the
 * first one can be fetched as a RevBlinkinLedDriver. docs/hardware-config.md
 * currently records this device as "Servo", which is either a doc error or a
 * config that needs changing; it is the first thing to check if this comes back
 * absent.
 *
 * OPTIONAL, like Deflector. If "blinkin" is missing, isPresent() is false and
 * every call here does nothing, so an op mode that uses LEDs still runs on a
 * robot that has not got them. LEDs are decoration and a status display -- they
 * should never be the reason a robot refuses to start a match.
 *
 * Usage:
 *     Leds leds = new Leds(hardwareMap);
 *     leds.set(BlinkinPattern.GREEN);        // a colour
 *     leds.readyToShoot();                   // or a meaning
 */
public class Leds {

    public static final String NAME = "blinkin";

    /** What the robot shows when nothing more interesting is happening. */
    public static BlinkinPattern IDLE = BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    /** Something is not zeroed or homed, so the robot is not ready. */
    public static BlinkinPattern NOT_READY = BlinkinPattern.RED;

    /** Flywheels at speed: safe to feed. */
    public static BlinkinPattern READY = BlinkinPattern.GREEN;

    /** Spinning up, homing, moving to a position -- working on it. */
    public static BlinkinPattern BUSY = BlinkinPattern.YELLOW;

    /** A ball is held. */
    public static BlinkinPattern LOADED = BlinkinPattern.BLUE;

    private final RevBlinkinLedDriver blinkin;

    /** The last pattern actually written, so we only write on a change. */
    private BlinkinPattern current = null;

    public Leds(HardwareMap hardwareMap) {
        blinkin = hardwareMap.tryGet(RevBlinkinLedDriver.class, NAME);
    }

    public boolean isPresent() {
        return blinkin != null;
    }

    /**
     * Show a pattern. Safe to call every loop: it only writes when the pattern
     * has actually changed, so a loop that sets the same thing 50 times a second
     * does not keep re-sending it.
     */
    public void set(BlinkinPattern pattern) {
        if (blinkin == null || pattern == null || pattern == current) return;
        blinkin.setPattern(pattern);
        current = pattern;
    }

    /** The pattern on display, or null if nothing has been set yet. */
    public BlinkinPattern get() {
        return current;
    }

    public void idle()         { set(IDLE); }
    public void notReady()     { set(NOT_READY); }
    public void readyToShoot() { set(READY); }
    public void busy()         { set(BUSY); }
    public void loaded()       { set(LOADED); }

    /** Lights out. */
    public void off() {
        set(BlinkinPattern.BLACK);
    }
}