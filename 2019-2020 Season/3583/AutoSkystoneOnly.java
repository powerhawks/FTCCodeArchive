package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone Only", group = "B Autos")

public class AutoSkystoneOnly extends AutoSkeletonSkystone {

    @Override
    public void runOpMode() {
        telemetry.addLine("Starting...");
        telemetry.update();
        red = true;
        initWebcam();
        initRobot();
        initGyro();
        initTeleGyro();
        TeleSkeletonSkystone.OFFSET = 0;
        telemetry.addLine("Ready to start!");
        telemetry.update();
        while(!opModeIsActive()){
            if(gamepad1.x){
                red = true;
            }else if(gamepad1.y){
                red = false;
            }
            if(red){
                telemetry.addLine("Alliance Side: Red");
            }else{
                telemetry.addLine("Alliance Side: Blue");
            }
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            findPattern();
            moveToFirstSkystone();
            pickUpSkystone();
            moveToPosition(-5, POWER);
            sleep(DELAY);
            turnWithGyro(QUARTERTURN, TURNPOWER);
            sleep(DELAY);
            depositySingleSkystone();
            requestOpModeStop();
        }

    }
}
