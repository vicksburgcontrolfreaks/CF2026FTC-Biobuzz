package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;

/**
 * Which robot is this code running on: 5618 or 6494?
 *
 * WHY THIS EXISTS
 * The two robots have different drivetrains: with the same motor directions,
 * 6494's rear wheels spin the opposite way from 5618's (found 2026-11-05).
 * Rebuilding one to match isn't an option, so the code has to know which
 * robot it's on. Asking a person to flip a setting before every deploy is how
 * a robot ends up driving backwards in a match, so we read it from the robot
 * itself instead.
 *
 * HOW IT KNOWS
 * Each robot's Control Hub has its own active hardware config, named after the
 * robot. We read that name. It only has to START with the robot number, so
 * "6494" and "6494-practice" both count as 6494.
 *
 * IF IT CAN'T TELL
 * It throws an error instead of guessing. A robot that refuses to INIT is a
 * 10-second fix in the pits (rename the config). A robot that guesses wrong
 * drives the wrong way on the field.
 */
public final class WhichRobot {

    public enum Robot { R5618, R6494 }

    private WhichRobot() {}

    /** The robot we're on. Throws IllegalStateException if the config name doesn't say. */
    public static Robot get() {
        String config = configName();
        if (config.startsWith("5618")) return Robot.R5618;
        if (config.startsWith("6494")) return Robot.R6494;
        throw new IllegalStateException(
                "Can't tell which robot this is: the active config is \"" + config + "\". "
                        + "Rename the config (Driver Station > Configure Robot) "
                        + "so it starts with 5618 or 6494.");
    }

    /** Just the number, for telemetry: "5618" or "6494". */
    public static String number() {
        return get() == Robot.R5618 ? "5618" : "6494";
    }

    /** The active hardware config's name, exactly as the Driver Station shows it. */
    public static String configName() {
        String name = new RobotConfigFileManager().getActiveConfig().getName();
        return name == null ? "" : name.trim();
    }
}
