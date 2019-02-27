package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Power Hawks Robotics on 12/12/2017.
 */

public class JAWLRelicArm3796 {

    DcMotor upDownTiltMotor;
    DcMotor leftRightMotor;
    DcMotor grabMotor;

    public JAWLRelicArm3796(DcMotor up, DcMotor left, DcMotor grab){

        upDownTiltMotor = up;
        leftRightMotor = left;
        grabMotor = grab;

        //Sets the behavior of the motors
        upDownTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }

    public void tiltUpOrDown(double power) {

        //Tilts the arm
        upDownTiltMotor.setPower(power);

    }

    public void moveLeftRight(double power) {

        //Extends the arm
        leftRightMotor.setPower(power);

    }

    public void grabRelic(double power) {

        //Opens the claw
        grabMotor.setPower(power);

    }

}
