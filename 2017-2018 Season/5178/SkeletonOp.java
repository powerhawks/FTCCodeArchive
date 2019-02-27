package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;



/**
 * Created by Tyler Lang of Clan Farquad on 12/31/2017.
 */


public abstract class SkeletonOp extends LinearOpMode{

    public  DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, SlideDrive,
            GrabDrive, relicRaise, relicSlide;

    public Servo jewlArm, relicSwing, relicGrab;

    public ColorSensor sensorOfColor;

    public RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;


//Mecanum Drive
    //    4 motorS
    static final String LEFT1NAME = "l1";
    static final String LEFT2NAME = "l2";
    static final String RIGHT1NAME = "r1";
    static final String RIGHT2NAME = "r2";

//Glyph Collection system
    //    2 motorS
    static final String GRABBER = "grabber";
    static final String SLIDER = "slide";

//Relic System
    //    2 motorS
    static final String RELICSLIDE = "rsl";
    static final String RELICRAISE = "rr";

    //    2 servoS
    static final String RELICGRAB = "rg";
    static final String RELICSWING = "rsw";

//Jewl arm
    //    one servo
    static final String JEWLARM = "ja";
    //    one coloR sensoR
    static final String COLORNAME = "cs";
    //    hsV valueS foR coloR sensoR
    float hsvValues[] = {0F,0F,0F};


    void goSlide(double power, long ms) throws InterruptedException {
        SlideDrive.setPower(power);
        Thread.sleep(ms);
        SlideDrive.setPower(0);
        Thread.sleep(400);
    }

    void goGrab(double power, long ms) throws InterruptedException {
        GrabDrive.setPower(power);
        Thread.sleep(ms);
        GrabDrive.setPower(0);
        Thread.sleep(400);
    }

    void stopMechanism() {
        SlideDrive.setPower(0.0);
        GrabDrive.setPower(0.0);
    }

    void stopDrive(){
        leftBackWheel.setPower(0);
        leftFrontWheel.setPower(0);
        rightBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
    }

    void stopRobot(){
        stopDrive();
        stopMechanism();
    }

    void goForward(double power, long ms) throws InterruptedException {
        // to go forward all motors go forward
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        rightFrontWheel.setPower(-power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }

    void goBackwards(double power, long ms) throws InterruptedException {
        //to go back, all motors spin backwards
        leftBackWheel.setPower(-1 * power);
        leftFrontWheel.setPower(-1 * power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }

    void goRight(double power, long ms) throws InterruptedException {
        //to go back, all motors spin backwards
        leftBackWheel.setPower(-1 * power);
        leftFrontWheel.setPower(1 * power);
        rightBackWheel.setPower(-1 * power);
        rightFrontWheel.setPower(1 * power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }

    void goLeft(double power, long ms) throws InterruptedException {
        //to go back, all motors spin backwards
        leftBackWheel.setPower(1 * power);
        leftFrontWheel.setPower(-1 * power);
        rightBackWheel.setPower(1 * power);
        rightFrontWheel.setPower(-1 * power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }

    void diagonalRightForward(double power, long ms) throws InterruptedException {
        //ro go diagonal to the right and forwards the
        //front left wheel and back right wheel move forward
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);

    }

    void diagonalRightBackward(double power, long ms) throws InterruptedException {
        //to go diagonal to the right and backwards the
        //front left wheel and back right wheel move backwards
        leftFrontWheel.setPower(-1 * power);
        rightBackWheel.setPower(-1 * power);

    }

    void diagonalLeftForward(double power, long ms) throws InterruptedException {
        //to go diagonal to the left and forwards the
        //front right wheel and back left wheel move forward
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(power);

    }

    void diagonalLeftBackwards(double power, long ms) throws InterruptedException {
        //to go diagonal to the left and backwards the
        //front right wheel and back left wheel move backwards
        rightFrontWheel.setPower(-1 * power);
        leftBackWheel.setPower(-1 * power);
    }

    void deployJewlArm() throws InterruptedException{
        jewlArm.setPosition(.89);
        Thread.sleep(1000);


    }

    void retractJewlArm() throws InterruptedException{
        jewlArm.setPosition(.95);
    }

    void updateColorSensor(){
        Color.RGBToHSV((sensorOfColor.red() * 255) / 800, (sensorOfColor.green() * 255) / 800,
                (sensorOfColor.blue() * 255) / 800, hsvValues);
    }

    // Jewel knocking movements

    void knockLeft(long ms) throws InterruptedException {
        leftFrontWheel.setPower(-1);
        leftBackWheel.setPower(-1);
        rightFrontWheel.setPower(-1);
        rightBackWheel.setPower(-1);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw e;
        } finally {
            stopRobot();
        }



    }

    void knockRight(long ms) throws InterruptedException {
        leftFrontWheel.setPower(1);
        leftBackWheel.setPower(1);
        rightFrontWheel.setPower(1);
        rightBackWheel.setPower(1);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw e;
        } finally {
            stopRobot();
        }
    }

    // hue methods

    boolean isRed(){
        updateColorSensor();
        float HUE = hsvValues[0];
        if(HUE > 330 || HUE < 30){
            return true;
        }
        else{
            return false;
        }

    }

    boolean isBlue(){
        updateColorSensor();
        float HUE = hsvValues[0];
        if (HUE < 270 && HUE > 190){
            return true;
        }
        else{
            return false;
        }
    }

    void rotateRight(double power, long ms) throws InterruptedException{
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }


    void rotateLeft(double power, long ms) throws InterruptedException{
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        Thread.sleep(ms);
        stopDrive();
        Thread.sleep(400);
    }

    void  moveWithColor()throws InterruptedException{
        if (isBlue()){
            knockLeft(100);
            retractJewlArm();
            Thread.sleep(250);
            knockRight(100);
            telemetry.addData("state", "Blue");
        }
        else if (isRed()){
            knockRight(100);
            retractJewlArm();
            Thread.sleep(250);
            knockLeft(100);
            telemetry.addData("state", "Red");
        }
        else{
            retractJewlArm();
            Thread.sleep(250);
            telemetry.addData("state", "noRead");
        }
    }

    void shiftRightCorner(double power, long ms) throws InterruptedException{
        leftFrontWheel.setPower(-1* power);
        rightFrontWheel.setPower(power);
    }

    void shiftleftCorner(double power, long ms) throws InterruptedException{
        leftFrontWheel.setPower(power);
        rightFrontWheel.setPower(-1 * power);
    }


}