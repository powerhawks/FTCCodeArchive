package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Power Hawks Robotics on 12/7/2017.
 */

public class PhoneWave_Autonomous extends LinearOpMode {
    //Initializes driving, shooting, and collecting motors for Evangelion Unit-01

// motors
    //Moves the robot
    public DcMotor lFrontDrive;
    public DcMotor lBackDrive;
    public DcMotor rFrontDrive;
    public DcMotor rBackDrive;
    //Raises the Glyph Lift
    public DcMotor gLift;

//Servos
    //Grabs the Glyphs
    public Servo lGClamp;
    public Servo rGClamp;
    //Lowers and raises the side jewel arm
    public Servo jewelArm;

//sensors
    public ColorSensor sightBud;
    float hsvValues[] = {0F,0F,0F};

//VUFORIA
//    (<.>_<.>) (<.>_<.>) (<.>_<.>) (<.>_<.>) (<.>_<.>) (<.>_<.>) (<.>_<.>)

    public void runOpMode()
    {
        initRobot();

        waitForStart();
        boolean autoMoveDone = false;
        while (opModeIsActive()) {
            if (!autoMoveDone) {
                lowerJewel();

            }
            idle();
        }
    }
    public void initRobot ()
    {
        lFrontDrive = hardwareMap.dcMotor.get("lFrontDrive");
        lBackDrive = hardwareMap.dcMotor.get("lBackDrive");
        rFrontDrive = hardwareMap.dcMotor.get("rFrontDrive");
        rBackDrive = hardwareMap.dcMotor.get("rBackDrive");
        lFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        gLift = hardwareMap.dcMotor.get("gLift");

        lGClamp = hardwareMap.servo.get("lGClamp");
        rGClamp = hardwareMap.servo.get("rGClamp");
        jewelArm = hardwareMap.servo.get("jewelArm");
    }

    public void budSee(String joel)
    {
        if(joel.equals("r") || joel.equals("red")){

        }
        else if(joel.equals("b") || joel.equals("blue")) {

        }
    }

    //Stops robot functions
    public void stop(long time)
    {
        try
        {
            Thread.sleep(time);
        }
        catch(Exception e) {}
    }

//Drives
    //Drives the robot forwards
    public void forward(long time)
    {
        lFrontDrive.setPower(1.0);
        lBackDrive.setPower(1.0);
        rFrontDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        stop(time);
        lFrontDrive.setPower(0.0);
        lBackDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
    }
    //Turns the robot 90 degrees left.
    public void turnLeft(long time)
    {
        lFrontDrive.setPower(-1.0);
        lBackDrive.setPower(-1.0);
        rFrontDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        stop(time);
        lFrontDrive.setPower(0.0);
        lBackDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
    }
    //Turns the robot 90 degrees right.
    public void turnRight(long time)
    {
        lFrontDrive.setPower(1.0);
        lBackDrive.setPower(1.0);
        rFrontDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        stop(time);
        lFrontDrive.setPower(0.0);
        lBackDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
    }
    //Runs the robot backwards.
    public void reverse(long time)
    {
        lFrontDrive.setPower(-1.0);
        lBackDrive.setPower(-1.0);
        rFrontDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        stop(time);
        lFrontDrive.setPower(0.0);
        lBackDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
    }
//Functions
    public void lowerJewel() {
        jewelArm.setPosition(0.36);
    }
    public void raiseJewel() {
        jewelArm.setPosition(0.0);
    }
    public void closeGlyphArm() {
        lGClamp.setPosition(0.3);
        rGClamp.setPosition(0.1);
    }
    public void openGlyphArm() {
        lGClamp.setPosition(0.02);
        rGClamp.setPosition(0.4);
    }
    //Returns jewel color from sightBud.
    public void getJoel() {

    }
    //Runs jewel Autonomous if on Blue Alliance.
    public void blueAlliance() {

    }
    //Runs jewel Autonomous if on Red Alliance.
    public void redAlliance() {

    }
}
