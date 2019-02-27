package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Power Hawks Robotics on 11/11/2017
 * <p>
 * November 11, 2017
 * Chase Galey and Joe Lewis
 * Jordan Alexander Wienstien Long Movement Methods
 */

public class JAWLDrive3796 {
    DcMotor leftDrive;
    DcMotor rightDrive;

    public JAWLDrive3796(DcMotor left, DcMotor right) {

        //Construct the drive variable for later use
        this.leftDrive = left;
        this.rightDrive = right;
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    //Method for setting encoder modes on both of our motors
    public void setEncoders(boolean bool) {
        if (bool) {
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Sets encoders to the STOP_AND_RESET_ENCODER runmode
    //Motors must be reset into the appropiate runmode after using this method
    public void resetEncoders() {
        /*leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());*/
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Sets the power for the right motor
    public void rightDrive(double power) {

        rightDrive.setPower(power);

    }

    //Sets the power for the left motor
    public void leftDrive(double power) {

        leftDrive.setPower(power);

    }


    //Autonomous Movement Functions


    public void forward(int ticks, double power) {
        int leftTargetPos = leftDrive.getCurrentPosition() + ticks;
        int rightTargetPos = rightDrive.getCurrentPosition() + ticks;
        leftDrive.setTargetPosition(leftTargetPos);
        rightDrive.setTargetPosition(rightTargetPos);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void backwards(int ticks, double power) {
        int leftTargetPos = leftDrive.getCurrentPosition() - ticks;
        int rightTargetPos = rightDrive.getCurrentPosition() - ticks;
        leftDrive.setTargetPosition(leftTargetPos);
        rightDrive.setTargetPosition(rightTargetPos);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    //This method will return an array of motor positions: {leftPosition, rightPosition}
    public int[] getPositions() {
        return new int[]{leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition()};
    }

    //ToDo: Maybe convert these to degrees

    //This will make the robot turn left during autonomous mode
    //THESE ARE UNTESTED
    /*public void turnLeft(int degrees, double power) {
        int ticks = 0;
        for(int i = 0; i < Math.abs(degrees); i++) {
            ticks += (1120/360);
        }

        if(degrees < 0) {
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);
        } else {
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);
        }
        rightDrive.setPower(power);
    }

    //This will make the robot turn right during autonomous mode
    public void turnRight(double degrees, double power) {
        int ticks = 0;
        for(int i = 0; i < Math.abs(degrees); i++) {
            ticks += (1120/360);
        }

        if(degrees < 0) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - ticks);
        } else {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
        }
        leftDrive.setPower(power);
    }*/

    public void turnLeft(int ticks, double power) {
        //Before this method was implemented we had an if statement checking if ticks was less than 0
        //If it was we subtracted the ticks from the current position. This ended up subtracting a negative number (adding it).
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void turnRight(int ticks, double power) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);
        leftDrive.setPower(power);
        rightDrive(power);
    }

    //This is another implementation of the method above
    //This method below will return busy even when the motor is within one tick of its target position, but it is still trying to get there
    /*public boolean isBusy() {
        if (leftDrive.isBusy() || rightDrive.isBusy()) {
            return true;
        } else {
            return false;
        }
    }*/

    //This method below will return false if both of the motors are within int variance of the target postions
    public boolean isBusy(int leftTarget, int rightTarget, int variance) {
        boolean leftIsBusy;
        boolean rightIsBusy;
        //This long and complicated if statement checks if the left motor's current position is within a range of variance from its target position
        if(leftTarget - variance < leftDrive.getCurrentPosition() && leftTarget + variance > leftDrive.getCurrentPosition()) {
            leftIsBusy = false;
        } else {
            leftIsBusy = true;
        }
        //The if statement below is the same as above, just applied for the right drive
        if(rightTarget - variance < rightDrive.getCurrentPosition() && rightTarget + variance > rightDrive.getCurrentPosition()) {
            rightIsBusy = false;
        } else {
            rightIsBusy = true;
        }

        if(leftIsBusy || rightIsBusy) {
            return true;
        } else {
            return false;
        }
    }
}