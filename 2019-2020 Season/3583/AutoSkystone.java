package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone", group = "B Autos")

public class AutoSkystone extends AutoSkeletonSkystone {

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
                phoneCam.setPipeline(patternPipelineRed);

            }else if(gamepad1.y){
                red = false;
                phoneCam.setPipeline(patternPipelineBlue);
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
            moveToFoundationFromFirstSkystone();
//            depositSkystone();
//            if(red){
//                turnWithGyro(QUARTERTURN, -TURNPOWER);
//            }else{
//                turnWithGyro(QUARTERTURN, -TURNPOWER);
//            }
//            moveFoundation();
            setLiftPosition(-4.7, 1);
            sleep(DELAY);
            grab.setPower(-1);
            sleep(DELAY);
            moveToPosition(1, POWER);
            if(red) {
                moveToPosition(-38, POWER);
            }else{
                moveToPosition(-38, POWER);
            }
            sleep(DELAY);
            setLiftPosition(4, 1);
            sleep(DELAY);
            if(red) {
                turnWithGyro(60, -TURNPOWER);
            }else{
                turnWithGyro(65, TURNPOWER);
            }
            tapeMeasure.setPower(1);
            while (opModeIsActive()){}
        }

    }
}
