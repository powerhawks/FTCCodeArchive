package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Name: Ramp driver class
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: November 27, 2018
 * */

public class KowallskiRamp3796 {

    //Initialize both motors. The extendMotor is the one that controls the extending and contracting
    //of the ramp, and the rampMotor is the one that controls the up and down movement of the ramp.
    DcMotor rampMotor, extendMotor;

    //Extension power modifier (Speed of the extension)
    static final double EXT_PWR_MOD = 1.0;
    //Ramp up power modifier (Speed of raising)
    static final double PWR_MOD = 1.0;
    //Ramp down power modifier (Speed of lowering)
    static final double DWN_PWR_MOD=1.0;
    //Maximum ramp extension in ticks. This is to stop the ramp from running off the track.
    static final int MAX_RAMP_EXT = 15000;

    //Constructor for the ramp
    public KowallskiRamp3796(DcMotor ramp, DcMotor extend){
        this.rampMotor = ramp;
        //Ensure that the ramp motor stays stationary when the controller is let go rather than falling
        this.rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extendMotor = extend;
    }

    //Reset the ramp's encoders to zero. This is to ensure that the ramp's stopper is as accurate as
    //it can be.
    public void resetRampMotor()
    {
        //Reset the encoder
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Enable the encoder (Just in case it isn't for whatever reason)
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Extend or Contract the ramp
    // (+) -> Extend
    // (-) -> Contract
    public void extendOrContract(double power)
    {
        //If the ramp is contracting, don't worry about the software stop
        //This allows the driver to retract the arm if they need to, even past the stop
        if(power < 0)
        {
            //Set the power based on the extension power modifier and the power parameter
            extendMotor.setPower(power * EXT_PWR_MOD);
        }else
        {
            //If the encoder's current position is less than the static variable which controls the
            //maximum amount, then it runs as normal
            if(extendMotor.getCurrentPosition() < MAX_RAMP_EXT)
            {
                //Set the power based on the extension power modifier and the power parameter
                extendMotor.setPower(power * EXT_PWR_MOD);
            } else {
                //If the encoder's current position is more than the maximum amount, then the ramp
                //stops extending
                extendMotor.setPower(0);
            }
        }

    }

    //Lifts or lowers the ramp. This is based off the rampMotor motor
    // (+) -> Lower
    // (-) -> Lift
    public void liftOrLower(double power){
        //This ensures that there can be a different power modifier for lifting and lowering (As per
        //request by the drive team)
        if (power < 0) {
            rampMotor.setPower(power * DWN_PWR_MOD);
        } else {
            rampMotor.setPower(power * PWR_MOD);
        }
    }

    //This is really just used for telemetry to notify the driver how far out the ramp is extended
    public int getTicks()
    {
        return extendMotor.getCurrentPosition();
    }

    //Stop the ramp up/down movement (Just sets the power to 0)
    public void stopMovementRmp(){
        rampMotor.setPower(0);
    }

    //Stop the ramp extend/contract movement (Just sets the power to 0)
    public void stopMovementExt() {
        extendMotor.setPower(0);
    }
}
