package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Power Hawks Robotics on 12/14/2017.
 */

/*
introduced diagonal movements by altering the front wheel and back
wheel on the opposite side
 */

@Autonomous(name = "AutoBlue1")
public class AutoBlue1 extends SkeletonOp {

    public void initRobot(){
        GrabDrive = hardwareMap.dcMotor.get("grabber");
        SlideDrive = hardwareMap.dcMotor.get("slide");
        leftFrontWheel = hardwareMap.dcMotor.get("l1");
        leftBackWheel = hardwareMap.dcMotor.get("l2");
        rightFrontWheel = hardwareMap.dcMotor.get("r1");
        rightBackWheel = hardwareMap.dcMotor.get("r2");
        jewlArm = hardwareMap.servo.get("ja");
        sensorOfColor = hardwareMap.colorSensor.get("cs");
    }
    @Override
    public void runOpMode() throws InterruptedException{
        initRobot();
        waitForStart();


        boolean autoMoveDone = false;

        try {
            while (opModeIsActive()) {
                if (!autoMoveDone) {
                    //DO NOT USE NEGATIVES FOR DRIVE, USE SEPARATE FUNCTION
                    goGrab(-0.75, 1650);
                    goSlide(.75,1200);
                    deployJewlArm();
                    Thread.sleep(500);
                    updateColorSensor();
                    if (isBlue()){
                        knockLeft(100);
                        retractJewlArm();
                        Thread.sleep(250);
                        knockRight(100);
                    }
                    else if (isRed()){
                        knockRight(100);
                        retractJewlArm();
                        Thread.sleep(250);
                        knockLeft(100);
                    }
                    else{
                        retractJewlArm();
                        Thread.sleep(250);

                    }

                    Thread.sleep(400);
                    retractJewlArm();

                    goBackwards(0.5, 1400);
                    rotateRight(1, 720  );
                    goForward(.25, 1100);
                    goGrab(0.75,300);

//                    Nothing Wrong with this
                    goBackwards(.25,300);
                    goForward(.25, 350);
                    goBackwards(.25,350);
                    goGrab(.65, 1650);
                    goSlide(-0.65,1200);
                    autoMoveDone = true;
                }
                idle();
            }
        }
        finally {
            stopRobot();
        }
    }
}
