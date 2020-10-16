package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Foundation", group = "B Autos")
public class AutoFoundation extends AutoSkeletonSkystone {
    @Override
    public void runOpMode() {
        initRobot();
        initGyro();
        initTeleGyro();
        TeleSkeletonSkystone.OFFSET = -90;
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
        while (opModeIsActive()) {
//            if(red){
//                moveToPosition(-20, POWER);
//            }else{
//                moveToPosition(17, POWER);
//                turnWithGyro(10, TURNPOWER);
//            }
            strafeToPosition(31, POWER);
            sleep(DELAY);
//            if(red) {
//                DriveRightCorrect(35);
//            }else{
//                DriveRightCorrect(31);
//            }
            gyroHold(.5, 0, 1);
            sleep(DELAY);
            drag.setPower(-1);
            sleep(DELAY);
            if(red) {
                DriveLeftCorrect(30);
            }else{
                DriveLeftCorrect(30);
            }
            sleep(DELAY);
            if (red) {
                turnWithGyro(110, TURNPOWER);
            }else{
                turnWithGyro(90, -TURNPOWER);
            }
            sleep(DELAY);
            drag.setPower(1);
            sleep(DELAY);
            if(red){
                turnWithGyro(35, TURNPOWER);
            }else{
                turnWithGyro(100, -TURNPOWER);
            }
            tapeMeasure.setPower(1);
            sleep(4000);
            requestOpModeStop();
        }
    }
}

/*

abstract class Auto() extends LinearOpMode{
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive())
        {
            run();
        }
    }
    void initialize() {
//        Initialize entire robot and all mechanisms
    }
    abstract void run() {
//        Do things
    }
}
class AutoThatDoesThings extends Auto {
    public void run() {
//        Do specific stuff
    }
}
 */