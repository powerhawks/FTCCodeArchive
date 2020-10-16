package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoSkeletonSkystone;
@Autonomous(name = "Test Auto", group = "A Tests")
public class TestAuto extends AutoSkeletonSkystone {
    @Override
    public void runOpMode() {
        initRobot();
        initGyro();
        waitForStart();
        while(opModeIsActive()){
            gyroHold(.75, 10, 30);
            requestOpModeStop();
        }
    }
}
