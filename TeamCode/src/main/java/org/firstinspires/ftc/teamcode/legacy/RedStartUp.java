package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auton.RouteAuto;
import org.firstinspires.ftc.teamcode.Auton.Routes;
import org.firstinspires.ftc.teamcode.field.Alliance;

/*
 * Red, starting against the wall our garden is on (the audience wall).
 * All the real code is in RouteAuto; the route is in Routes.
 */
@Disabled
@Autonomous(name = "Red Start Up (Garden Wall)", group = "Autonomous")
public class RedStartUp extends RouteAuto {
    public RedStartUp() {
        super(Alliance.RED, Routes.startUpGardenWall(Routes.Park.NONE));
    }
}
