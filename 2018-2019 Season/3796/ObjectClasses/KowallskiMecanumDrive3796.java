package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.MotorSide;
import org.firstinspires.ftc.teamcode.Enums.diag;

/**
 * Name: Mecanum drive class
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: October 6, 2018
 * */

public class KowallskiMecanumDrive3796 {

    //This isn't really used.. This was an attempt at encoder driving but was never really used due
    //to errors with the Linear Op Mode and Autonomous so all of the methods that would drive this
    //are in the KowallskiAutonomousDropDown3796 class but we're going to leave it in here just in
    //case we ever do need it
    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TICKS_PER_INCH_L          = (5122.0/120);
    static final double     TICKS_PER_INCH_R          = (5133.0/120);
    static final double     DISTANCE_BETWEEN_WHEELS   =  11.40;

    //Limit of the power for the robot, this is just to stop the robot from skidding
    static final double POWER_LIM = 0.85;
    //If we want to turn off the power limit we can just set this to false rather than changing many
    //lines of code
    static final boolean POWER_LIM_ENABLE = true;
    //Scalar for the power of the robot (Set to 1 to make it drive as fast as possible)
    static final double powerScalar = 1.0;
    //Power for the autonomous (Keeps it consistent)
    static double AUTO_POWER = 0.25;

    //Initialize the DcMotors (Public because we access them in the autonomous encoder methods)
    public DcMotor rightFrontWheel, rightBackWheel, leftFrontWheel, leftBackWheel;

    //Constructor (It just sets the motors to whatever you input)
    public KowallskiMecanumDrive3796(DcMotor rightFront, DcMotor rightBack, DcMotor leftFront, DcMotor leftBack){
        this.rightFrontWheel = rightFront;
        this.rightBackWheel = rightBack;
        this.leftFrontWheel = leftFront;
        this.leftBackWheel = leftBack;
    }

    //These methods are just made to centralize the code and make it so we can change the minute details
    //of the workings of the motors with just one change. This is essentially just putting the power in,
    //but also setting a maximum power if the maximum power is enabled.
    public void moveFrontRightWheel(double power){
        //Limit the power if the power limit is enabled and the power is over the limit
        if((power > POWER_LIM) && POWER_LIM_ENABLE){power = POWER_LIM;}
        else if((power < -POWER_LIM) && POWER_LIM_ENABLE){power = -POWER_LIM;}

        rightFrontWheel.setPower(power * powerScalar);
    }

    public void moveBackRightWheel(double power){
        //Limit the power if the power limit is enabled and the power is over the limit
        if((power > POWER_LIM) && POWER_LIM_ENABLE){power = POWER_LIM;}
        else if((power < -POWER_LIM) && POWER_LIM_ENABLE){power = -POWER_LIM;}

        rightBackWheel.setPower(power * powerScalar);
    }

    public void moveFrontLeftWheel(double power){
        //Limit the power if the power limit is enabled and the power is over the limit
        if((power > POWER_LIM) && POWER_LIM_ENABLE){power = POWER_LIM;}
        else if((power < -POWER_LIM) && POWER_LIM_ENABLE){power = -POWER_LIM;}

        leftFrontWheel.setPower(power * powerScalar);
    }

    public void moveBackLeftWheel(double power){
        //Limit the power if the power limit is enabled and the power is over the limit
        if((power > POWER_LIM) && POWER_LIM_ENABLE){power = POWER_LIM;}
        else if((power < -POWER_LIM) && POWER_LIM_ENABLE){power = -POWER_LIM;}

        leftBackWheel.setPower(power * powerScalar);
    }

    //Drive the robot forward (This uses all four motors and is again, to centralize the code)
    //When you move the joystick the other way, it will reverse the direction of movement, so we do not
    //need to write two methods for this.
    // (+) -> Forward
    // (-) -> Backward
    public void forwardBackward(double power)
    {
        this.moveBackLeftWheel((powerScalar * power));
        this.moveBackRightWheel(-(powerScalar * power));
        this.moveFrontLeftWheel((powerScalar * power));
        this.moveFrontRightWheel(-(powerScalar * power));
    }

    //Drives the robot left or right (This uses all four motors and again, is to centralize the code)
    //When you move the joystick the other way, it will reverse the direction of movement, so we do not
    //need to write two methods for this.
    // (+) -> Left
    // (-) -> Right
    public void leftRight(double power)
    {
        this.moveBackLeftWheel(-(power * powerScalar));
        this.moveBackRightWheel(-(power * powerScalar));
        this.moveFrontLeftWheel((power * powerScalar));
        this.moveFrontRightWheel((power * powerScalar));
    }

    //Drives the robot diagonally (This isn't really used much but it's always nice to have)
    //This takes in the dir enumeration for the direction it's in, and drives the robot based on it.
    public void diagonal(diag dir)
    {
        //Switch statement because I like those
        switch(dir)
        {
            //These will check which enum dir has been set to. We only have two motors move because
            //of the way that Mecanum wheels work.
            case upleft:
                //Moving the FrontRight wheel and the BackLeft wheel forwards will make the robot slide
                //diagonally front-left because of how the Mecanum wheels work.
                this.moveBackLeftWheel((powerScalar));
                this.moveFrontRightWheel(-(powerScalar));
                break;
            case upright:
                //Moving the FrontLeft wheel and the BackRight wheel forwards will make the robot slide
                //diagonally front-right because of the Mecanum wheels.
                this.moveBackRightWheel(-(powerScalar));
                this.moveFrontLeftWheel((powerScalar));
                break;
            case downleft:
                //Moving the FrontLeft wheel and the BackRight wheel backwards will make the robot slide
                //diagonally back-left because of the way the Mecanum wheels are made.
                this.moveBackRightWheel((powerScalar));
                this.moveFrontLeftWheel(-(powerScalar));
                break;
            case downright:
                //Moving the FrontRight wheel and the BackLeft wheel backwards will make the robot slide
                //diagonally back-right because of the angle of the Mecanum wheels.
                this.moveBackLeftWheel(-(powerScalar));
                this.moveFrontRightWheel((powerScalar));
                break;
            default:
                //This is just in case something goes wrong and the user inputs a weird diagonal method..
                //Doesn't really serve a purpose but is there just in case
                break;
        }
    }

    //This turns the robot left or right based on the power input
    //Applying the same power to all the motors will make them all spin the same direction, clockwise or counterclockwise.
    //Since two of them are in the opposite orientation, this will make the robot spin.
    // (+) -> Turn Left
    // (-) -> Turn Right
    public void turn(double power){
        //Half the power because the robot doesn't like full power
        power/=2;

        //Set the power (Using the earlier methods to centralize the code even more)
        moveBackLeftWheel(power);
        moveBackRightWheel(power);
        moveFrontLeftWheel(power);
        moveFrontRightWheel(power);
    }

    //This stops the movement of all four motors and essentially stops the robot in its place. This doesn't 100% stop
    //the robot, however, due to slippage and torque issues. This sets all of the motors to zero essentially and makes
    //programming a lot easier.
    public void stopMovement()
    {
        //Set all of the powers to zero.
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
        leftFrontWheel.setPower(0);
    }

    //NOTE: THE METHODS BELOW AREN'T REALLY USED DUE TO ISSUES WITH OPMODE AND THINGS OF THAT NATURE.
    //THEY ARE HERE JUST IN CASE WE DO NEED TO USE THESE FOR WHATEVER REASON, BUT WE SHOULDN'T EVER
    //NEED TO USE THEM.


    //If we need to make sure that no motors are moving at all what so ever, we can use this method.
    //This is meant for the encoder based driving system.
    // (Moving)        -> True
    // (Not moving)    -> False
    public boolean busy()
    {
        //Both of them are busy
        //AKA: Left wheel and Right wheel moving
        return leftBackWheel.isBusy() || rightBackWheel.isBusy();
    }

    //Converts inches to ticks in order to make the tick driving a lot more human-readable
    public int inchesToTicks(double inches, MotorSide side)
    {
        //Ensures that the distance is being read correctly (The distance between the wheels makes
        //the values completely off sometimes)
        inches -= DISTANCE_BETWEEN_WHEELS;
        switch(side)
        {
            //This is based off what motor the user wants to get it off, as both motors have a
            //different amount of ticks per inch
            case LeftMotor:
                return (int)((inches * TICKS_PER_INCH_L));
            case RightMotor:
                return (int)((inches * TICKS_PER_INCH_R));
            default:
                return -1;
        }

    }

    //Converts ticks to inches so we can view the robot's current distances easier.
    public double ticksToInches(int ticks, MotorSide side)
    {
        //Convert it to a double to prevent rounding errors
        double t = (double)ticks;
        switch(side)
        {
            //Ensures that the distance is being read correctly (The distance between the wheels makes
            //the values completely off sometimes)
            case LeftMotor:
                return (int)((t / TICKS_PER_INCH_L));
            case RightMotor:
                return (int)((t / TICKS_PER_INCH_R));
            default:
                return -1;
        }
    }

    //Set all of the encoders based on what mode the program is running
    public void setEncoders(boolean set)
    {
        if(set) {
            //Reverse the right side
            rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
            rightBackWheel.setDirection(DcMotor.Direction.REVERSE);

            //Reset the encoders to be in the mode for running encoder-based autonomous
            leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else{
            //Run the regular/Tele-Op mode

            //Make the right side back to forward mode
            rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
            rightBackWheel.setDirection(DcMotor.Direction.FORWARD);

            //Set the back wheels to not use encoders as they are not needed for Tele-Op
            leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

}