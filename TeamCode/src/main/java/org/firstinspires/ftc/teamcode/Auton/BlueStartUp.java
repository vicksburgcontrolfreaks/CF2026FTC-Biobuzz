package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Blue, starting against the wall our garden is on (the rear wall).
 * Same as Red Start Up, spun 180 degrees.
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Autonomous(name = "Blue Start Up (Garden Wall)", group = "Autonomous")
public class BlueStartUp extends RouteAuto {
    public BlueStartUp() {
        super(Alliance.BLUE, Routes.START_UP_GARDEN_WALL);
    }
}
