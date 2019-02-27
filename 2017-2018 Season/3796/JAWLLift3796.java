package org.firstinspires.ftc.teamcode;

/*
* November 14, 2017
* Created by Chase Galey and Joe Lewis
* Tony the Train that Tried Lift Component class
*
*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

class JAWLLift3796 {

    private DcMotor liftMotor;
    //private int bottomPosition;
    //private int topPosition;

    public JAWLLift3796(DcMotor lift) {
        //Set instance motor to our lift motor
        this.liftMotor = lift;
        //Set motor behavior to run without encoder
        this.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set zero power behavior so that the motor holds its position
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Set instance speedLimit to provided speedlimit
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        /*//Assuming that the lift starts at the very bottom, set the bottom postion to the current position and current revolution to 0
        bottomPosition = liftMotor.getCurrentPosition();



        //With a 3.5 in diameter wheel that needs to move 12 inches
        //More information in calculations file {@link Calculations}
        double circumference = Math.PI * 3.5;
        //Revolutions needed to lift the lift to max height
        double neededRevolutions = 12 / circumference;
        //Needed ticks to move the motor needed revolutions
        double neededTicks = 1680 * neededRevolutions;

        topPosition = bottomPosition + (int) neededTicks;*/

    }

    //Move motor with fine control over motor speed
    //To be used during teleop
    public void moveMotor(double power) {
        liftMotor.setPower(power);

    }
}