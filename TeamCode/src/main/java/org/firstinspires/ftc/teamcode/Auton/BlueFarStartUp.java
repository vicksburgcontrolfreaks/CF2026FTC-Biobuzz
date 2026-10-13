package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Blue, starting against the far wall (the audience wall, beside a flower).
 * Same as Red Far Start Up, spun 180 degrees.
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Autonomous(name = "Blue Start Up (Far Wall)", group = "Autonomous")
public class BlueFarStartUp extends RouteAuto {
    public BlueFarStartUp() {
        super(Alliance.BLUE, Routes.START_UP_FAR_WALL);
    }
}
