package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auton.RouteAuto;
import org.firstinspires.ftc.teamcode.Auton.Routes;
import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Red, starting against the far wall (the rear wall, beside a flower).
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Disabled
@Autonomous(name = "Red Start Up (Far Wall)", group = "Autonomous")
public class RedFarStartUp extends RouteAuto {
    public RedFarStartUp() {
        super(Alliance.RED, Routes.startUpFarWall(Routes.Park.NONE));
    }
}
