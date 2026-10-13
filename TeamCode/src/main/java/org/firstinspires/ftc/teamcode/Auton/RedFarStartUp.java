package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Red, starting against the far wall (the rear wall, beside a flower).
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Autonomous(name = "Red Start Up (Far Wall)", group = "Autonomous")
public class RedFarStartUp extends RouteAuto {
    public RedFarStartUp() {
        super(Alliance.RED, Routes.START_UP_FAR_WALL);
    }
}
