package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * ONE auto for every match: pick the alliance, route, wait, and park on
 * gamepad 1 during INIT (see AutoMenu for the buttons), lock it in with A,
 * then press Start.
 *
 * Adding a setting:
 *   1. Add a line to the menu below.
 *   2. Use its .get() in applyMenu().
 *
 * Adding a route: write it in Routes, then add it to AutoRoute below.
 */
@Autonomous(name = "Select Auto", group = "Autonomous")
public class SelectAuto extends RouteAuto {

    // The routes you can pick. Each one knows where it starts.
    enum AutoRoute {
        GARDEN_WALL("Garden Wall"),
        FAR_WALL("Far Wall"),
        GARDEN_WALL_LAUNCH("Garden Wall + Launch"),
        FAR_WALL_YIELD("Far Wall + Yield");

        final String label;

        AutoRoute(String label) {
            this.label = label;
        }

        Route route(Routes.Park park) {
            switch (this) {
                case GARDEN_WALL:        return Routes.startUpGardenWall(park);
                case FAR_WALL:           return Routes.startUpFarWall(park);
                case GARDEN_WALL_LAUNCH: return Routes.gardenWallLaunch(park);
                case FAR_WALL_YIELD:     return Routes.farWallYield(park);
                default: throw new IllegalStateException("No route for " + this);
            }
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
    private final AutoMenu.Choice<AutoRoute> autoRoute =
            menu.add("Route", AutoRoute.GARDEN_WALL, AutoRoute.FAR_WALL,
                    AutoRoute.GARDEN_WALL_LAUNCH, AutoRoute.FAR_WALL_YIELD);
    private final AutoMenu.Choice<Integer> startWait =
            menu.add("Wait (sec)", 0, 3, 5, 8);
    private final AutoMenu.Choice<Routes.Park> park =
            menu.add("Park", Routes.Park.GARDEN_SIDE, Routes.Park.FAR_SIDE, Routes.Park.NONE);
    // Only used by routes with a yield step (Far Wall + Yield). Seconds after Start.
    private final AutoMenu.Choice<Integer> yieldUntil =
            menu.add("Yield until (sec)", 15, 18, 20, 22, 25);

    // Hand the menu's current picks to RouteAuto.
    private void applyMenu() {
        useRoute(alliance.get(), autoRoute.get().route(park.get()), startWait.get());
        setYieldUntil(yieldUntil.get());
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
