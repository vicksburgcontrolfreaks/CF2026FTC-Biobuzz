package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auton.RouteAuto;
import org.firstinspires.ftc.teamcode.Auton.Routes;
import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Blue, starting against the far wall (the audience wall, beside a flower).
 * Same as Red Far Start Up, spun 180 degrees.
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Disabled
@Autonomous(name = "Blue Start Up (Far Wall)", group = "Autonomous")
public class BlueFarStartUp extends RouteAuto {
    public BlueFarStartUp() {
        super(Alliance.BLUE, Routes.startUpFarWall(Routes.Park.NONE));
    }
}
