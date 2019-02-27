package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Power Hawks Robotics on 12/14/2017.
 */

/*
introduced diagonal movements by altering the front wheel and back
wheel on the opposite side
 */

@Autonomous(name = "MethodTesting")
public class MethodTestV23 extends SkeletonOp {

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
                    deployJewlArm();
                    Thread.sleep(500);
                    updateColorSensor();
                    moveWithColor();
                    telemetry.addData("Did We Make It Through" , "YES");
                    Thread.sleep(400);
                    retractJewlArm();
                    telemetry.addData("Did We Make It To The End" , "YES");
                    telemetry.update();
                }
                idle();
            }
        }
        finally {
            stopRobot();
        }
    }
}
