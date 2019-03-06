package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Name: Mineral Arm controller
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: November 10, 2018
 * */

public class KowallskiMineralArm3796 {

    //Left and right arm motor
    //NOTE: THESE MOTORS SHOULD NEVER NOT MOVE AT THE SAME TIME. IF THEY DO, THE GEAR WILL JUST NOT
    //WORK AT ALL AND MIGHT BREAK
    DcMotor rightArmMotor, leftArmMotor;

    //Scalar for how much power should be applied
    private static double PWR_SCALAR = 1.0;

    //Initialize the Mineral arm (The motors are retrieved from the hardware map)
    public KowallskiMineralArm3796(DcMotor armRight, DcMotor armLeft){
        this.rightArmMotor = armRight;
        this.leftArmMotor = armLeft;
    }

    // Move the arm up or down
    // (+) -> Up
    // (-) -> Down
    public void moveUpOrDown(double power){
        //The arm will move up when the power is positive and down when it is negative
        rightArmMotor.setPower(power * PWR_SCALAR);
        leftArmMotor.setPower(-power * PWR_SCALAR);

    }

    //This will move the arm towards the robot slowly enough to apply a braking mechanism so the robot will not crash down.
    public void slowlyLower(){
        //Uses the above method to ensure abstraction
        this.moveUpOrDown(0.70 * PWR_SCALAR);
    }

    //Stop the movement of the arm (Power scalar doesn't need to be there but that's just to be sure)
    public void stopMovement(){
        leftArmMotor.setPower(0 * PWR_SCALAR);
        rightArmMotor.setPower(0 * PWR_SCALAR);
    }

}
