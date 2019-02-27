/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by Tyler Lang of Clan Farquad* on 12/23/2017.
 */

/*
more motor controller fixings
 */
@TeleOp (name="TestTeleOpV5")
public class TeleOpvFinal extends OpMode {

    private static final double ACCEPTINPUTTHRESHOLD = 0.15;
    private static double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train


    private  DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, SlideDrive,
            GrabDrive, relicRaise, relicSlide;
    private  Servo jewlArm, relicSwing, relicGrab;
    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get(SkeletonOp.LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(SkeletonOp.LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(SkeletonOp.RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(SkeletonOp.RIGHT2NAME);
        SlideDrive = hardwareMap.dcMotor.get(SkeletonOp.SLIDER);
        GrabDrive = hardwareMap.dcMotor.get(SkeletonOp.GRABBER);
        relicRaise = hardwareMap.dcMotor.get(SkeletonOp.RELICRAISE);
        relicSlide = hardwareMap.dcMotor.get(SkeletonOp.RELICSLIDE);

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        jewlArm = hardwareMap.servo.get(SkeletonOp.JEWLARM);
        relicSwing = hardwareMap.servo.get(SkeletonOp.RELICSWING);
        relicGrab = hardwareMap.servo.get(SkeletonOp.RELICGRAB);
    }

    @Override
    public void loop() {


        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
        double inputX = Math.abs(-gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x : 0;
        double inputC = Math.abs(gamepad1.left_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x: 0;

        SlideDrive.setPower(-gamepad2.left_stick_y*0.75);
        GrabDrive.setPower(gamepad2.right_stick_x);

        if(gamepad2.dpad_up) {
            jewlArm.setPosition(.89);
        }
        else{
            jewlArm.setPosition(.94);
        }

//        if(gamepad2.dpad_down) {
//            relicRaise.setPower(-1);
//        }
//        else{
//            relicRaise.setPower(0);
//        }
//
//        if(gamepad2.dpad_right) {
//            relicSlide.setPower(0.5);
//        }
//
//        if(gamepad2.dpad_left) {
//            relicSlide.setPower(-0.5);
//        }



        arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);



    }


    //This is the method that
    // y - forwards
    // x - side
    // c - rotation
public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
    double leftFrontVal = y + x + c;
    double rightFrontVal = y - x - c;
    double leftBackVal = y - x + c;
    double rightBackVal = y + x - c;

    //Move range to between 0 and +1, if not already
    double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
    Arrays.sort(wheelPowers);
    if (wheelPowers[3] > 1) {
        leftFrontVal /= wheelPowers[3];
        rightFrontVal /= wheelPowers[3];
        leftBackVal /= wheelPowers[3];
        rightBackVal /= wheelPowers[3];
    }
    double scaledPower = SCALEDPOWER;

    leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
    rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
    leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
    rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));

}



    public static void tylersMecnaum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack){
        // for now were going to ignore c and all of that just want to get it moving forward and backwards



    }
}