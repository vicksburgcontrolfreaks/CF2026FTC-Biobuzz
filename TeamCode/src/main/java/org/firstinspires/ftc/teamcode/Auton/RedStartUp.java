package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Red, starting against the wall our garden is on (the audience wall).
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Autonomous(name = "Red Start Up (Garden Wall)", group = "Autonomous")
public class RedStartUp extends RouteAuto {
    public RedStartUp() {
        super(Alliance.RED, Routes.START_UP_GARDEN_WALL);
    }
}
