package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Double Skystone", group = "B Autos")

public class AutoDoubleSkystone extends AutoSkeletonSkystone {

    @Override
    public void runOpMode() {
        telemetry.addLine("Starting...");
        telemetry.update();
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
			moveUnderBridgeFromFirstSkystone();
			depositSkystoneUnderBridge();
			moveToSecondSkystone();
			pickUpSkystone();
			if(red){
                turnWithGyro(90, TURNPOWER);
            }else{
			    turnWithGyro(90, -TURNPOWER);
            }
			sleep(DELAY);
			moveToPosition(70, POWER);
            sleep(DELAY);
            moveToPosition(-10, POWER);
            requestOpModeStop();
        }
    }
}
