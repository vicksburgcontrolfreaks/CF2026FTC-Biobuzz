package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * ONE auto for every match: pick the alliance, start position, and wait on
 * gamepad 1 during INIT (see AutoMenu for the buttons), lock it in with A,
 * then press Start.
 *
 * Adding a setting:
 *   1. Add a line to the menu below.
 *   2. Use its .get() in applyMenu().
 *
 * Adding a start position: add it to StartPosition with its route from Routes.
 */
@Autonomous(name = "Select Auto", group = "Autonomous")
public class SelectAuto extends RouteAuto {

    // Where the robot is placed, and which route it runs from there.
    enum StartPosition {
        GARDEN_WALL("Garden Wall", Routes.START_UP_GARDEN_WALL),
        FAR_WALL("Far Wall", Routes.START_UP_FAR_WALL);

        final String label;
        final Route route;

        StartPosition(String label, Route route) {
            this.label = label;
            this.route = route;
        }

        @Override
        public String toString() {
            return label; // what the menu shows
        }
    }

    // ---------------- The menu (first choice = default) ----------------
    private final AutoMenu menu = new AutoMenu();
    private final AutoMenu.Choice<Alliance> alliance =
            menu.add("Alliance", Alliance.RED, Alliance.BLUE);
    private final AutoMenu.Choice<StartPosition> startPosition =
            menu.add("Start", StartPosition.GARDEN_WALL, StartPosition.FAR_WALL);
    private final AutoMenu.Choice<Integer> startWait =
            menu.add("Wait (sec)", 0, 3, 5, 8);

    // Hand the menu's current picks to RouteAuto.
    private void applyMenu() {
        useRoute(alliance.get(), startPosition.get().route, startWait.get());
    }

    @Override
    public void init() {
        super.init();
        applyMenu();
    }

    @Override
    public void init_loop() {
        if (menu.update(gamepad1)) {
            applyMenu(); // something changed -- rebuild the paths and recheck them
        }
        menu.show(telemetry);
        telemetry.addLine("");
        addRouteTelemetry();
        telemetry.update();
    }
}
