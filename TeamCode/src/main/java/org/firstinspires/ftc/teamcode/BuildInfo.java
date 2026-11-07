package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shows WHICH CODE is on the robot: branch, commit, and when it was built.
 *
 * WHY THIS EXISTS
 * On 2026-11-05 a build from an old branch got deployed by accident. It had the
 * wrong drive directions and crashed on a missing color sensor, and nothing on
 * the Driver Station said which code was running. Now every important op mode
 * shows a line like this at INIT:
 *
 *     Build   main @ c904ce9  (built 11/05 19:42)
 *     Robot   6494
 *
 * If it says the wrong branch, or "+UNCOMMITTED", stop and redeploy from the
 * right branch. "+UNCOMMITTED" means the build had changes nobody committed,
 * so nobody can rebuild that exact code later.
 *
 * The values are filled in by TeamCode/build.gradle when the app is built.
 *
 * HOW TO USE IN AN OP MODE
 * Call BuildInfo.show(telemetry) in init, before telemetry.update().
 * New op modes that run on the robot should do this too.
 */
public final class BuildInfo {

    private BuildInfo() {}

    /** e.g. "main @ c904ce9  (built 11/05 19:42)" */
    public static String line() {
        return BuildConfig.GIT_BRANCH + " @ " + BuildConfig.GIT_COMMIT
                + (BuildConfig.GIT_DIRTY ? " +UNCOMMITTED" : "")
                + "  (built " + BuildConfig.BUILD_TIME + ")";
    }

    /**
     * Adds the Build and Robot lines. Never throws: if the robot can't be
     * identified it says so here, and the code that needs the robot (drive
     * setup) still refuses to run with its own clear error.
     */
    public static void show(Telemetry telemetry) {
        telemetry.addData("Build", line());
        String robot;
        try {
            robot = WhichRobot.number();
        } catch (IllegalStateException e) {
            robot = "UNKNOWN - config \"" + WhichRobot.configName() + "\"";
        }
        telemetry.addData("Robot", robot);
    }
}
