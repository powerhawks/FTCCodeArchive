package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//Created by Team 3583 on 10/26/2017

public abstract class SkeletonOpMode extends LinearOpMode
{
    //Initializes driving, shooting, and collecting motors for Evangelion Unit-01
    //EVA's motors
    public DcMotor lDrive;
    public DcMotor rDrive;
    public DcMotor shooter;
    public DcMotor collect;

    // motors
    //Moves the robot
    public DcMotor lBackDrive;
    public DcMotor rBackDrive;
    //Raises the Glyph Lift
    public DcMotor gLift;

    //servos
    //Grabs the Glyphs
    public Servo lGClamp;
    public Servo rGClamp;
    //Lowers and raises the side jewel arm
    public Servo jewelArm;

    //sensors
    public ColorSensor sightBud;
    float hsvValues[] = {0F,0F,0F};

    // inch movement methods
    //wheel radius
    static double WRadius = 1.9375;
    static double WCirc = (2 * 3.141592) * WRadius;
    // input:output, GR > 1 for torque, GR < 1 for speed
    static double GearRatio = 1;

    //////////////////////////////////////////// POSSIBLE ERROR: CHANGE TicksPerRev
    static int TicksPerRevolution = 1680;

    // robot radius from turn point to wheels
    static double RRadius = 8.25;
    static double RCirc = (2 * 3.141592) * RRadius;
    static int tolerance = 10;

    int ticksRight = 0;
    int ticksLeft = 0;

    public void initRobot(){
        lBackDrive = hardwareMap.dcMotor.get("lBackDrive");
        rBackDrive = hardwareMap.dcMotor.get("rBackDrive");
        lBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gLift = hardwareMap.dcMotor.get("gLift");
        gLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lGClamp = hardwareMap.servo.get("lGClamp");
        rGClamp = hardwareMap.servo.get("rGClamp");
        jewelArm = hardwareMap.servo.get("jewelArm");
        raiseJewel();
    }


    void sleep(int milli){
        try{
            Thread.sleep(milli);
        }
        catch(Exception e){
        }
    }

    double convertToTicks(double inch){

        return (inch / WCirc) * TicksPerRevolution * GearRatio;
    }
    // tick methods
    void fixPos(){
        int rightDiff = ticksRight - rBackDrive.getCurrentPosition() ;
        int leftDiff = ticksLeft - lBackDrive.getCurrentPosition() ;

        //boolean right = true;
        //boolean left = true;

        boolean RightAdd = false;
        boolean RightSub = false;
        boolean LeftAdd = false;
        boolean LeftSub = false;

        if(rightDiff > tolerance){
            RightAdd = true;
        }

        if(rightDiff < tolerance * -1){
            RightSub = true;
        }

        if(leftDiff > tolerance){

        }

        if(leftDiff < tolerance * -1){

        }
        else{
            //telemetry.addData("no Adjustments");
            //telemetry.update();
        }


        if (RightAdd && LeftAdd){
            forward(rightDiff);
            // forwardPos
        }
        else if (RightSub && LeftSub){
            backward(rightDiff * -1);
            // backward
        }
        else if (RightAdd && LeftSub){
            turnLeft(rightDiff);
            //turn left
        }
        else if (RightSub && LeftAdd){
            turnRight(leftDiff);
            //turn right
        }
        else if (RightAdd || RightSub){
            rightAdvance(rightDiff);
        }
        else if (LeftAdd || LeftSub){
            leftAdvance(leftDiff);
        }
        else{

        }
    }

    //accepts negatives
    void leftAdvance(double ticks){
        int init = lBackDrive.getCurrentPosition();

        if(ticks >= 0){
            lBackDrive.setPower(0.5);
            lBackDrive.setPower(0.5);
        }
        else if (ticks <= 0){
            lBackDrive.setPower(-0.5);
        }
        while(lBackDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();
    }

    void rightAdvance(double ticks){
        int init = rBackDrive.getCurrentPosition();

        if(ticks >= 0){
            rBackDrive.setPower(0.5);
        }
        else if (ticks <= 0){
            rBackDrive.setPower(-0.5);
        }

        while(rBackDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();
    }

    void forward(double ticks){
        //use: All
        //reference: one motor
        ticksRight += ticks;
        ticksLeft += ticks;

        int init = lBackDrive.getCurrentPosition();
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        while(lBackDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();

    }
    void backward(double ticks){
        //use:All
        //reference: one motor
        ticksRight -= ticks;
        ticksLeft -= ticks;

        int init = lBackDrive.getCurrentPosition();
        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        while(lBackDrive.getCurrentPosition() - init * -1 < ticks){
            sleep(10);
        }
        stopMotors();

    }

    void turnRight(double ticks){
        //use:All
        //reference: two motors?

        // negative right, positive left
        ticksRight -= ticks;
        ticksLeft += ticks;

        int init2 = rBackDrive.getCurrentPosition();
        int init1 = lBackDrive.getCurrentPosition();

        lBackDrive.setPower(1.0);
        rBackDrive.setPower(-1.0);

        while((lBackDrive.getCurrentPosition() - init1 < ticks) && (rBackDrive.getCurrentPosition() - init2 * -1 < ticks)){
            sleep(10);                                                                         // possible edit: remove * -1
        }
        stopMotors();
    }
    void turnLeft(double ticks){
        //use:All
        //reference: two motors?

        //positive right, negative left
        ticksRight += ticks;
        ticksLeft -= ticks;

        int init2 = rBackDrive.getCurrentPosition();
        int init1 = lBackDrive.getCurrentPosition();

        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(1.0);

        while((rBackDrive.getCurrentPosition() - init2 < ticks) && (lBackDrive.getCurrentPosition() - init1 * -1 < ticks)){
            sleep(10);                                                                       // possible edit: remove * -1
        }
        stopMotors();
    }

    void stopMotors(){
        lBackDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
    }

    // inch methods (More Advanced)
    void forwardInch(double inch){
        double ticks = convertToTicks(inch);
        forward(ticks);
    }
    void backwardsInch(double inch){
        double ticks = convertToTicks(inch);
        backward(ticks);
    }

    //Turning degrees?
    void turnRightInch(double deg){
        double inches = (deg / 360) * RCirc;
        double ticks = convertToTicks(inches);
        turnRight(ticks);
    }
    void turnLeftInch(double deg){
        double inches = (deg / 360) * RCirc;
        double ticks = convertToTicks(inches);
        turnLeft(ticks);
    }

    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800, (sightBud.blue() * 255) / 800, hsvValues);
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
        if (HUE < 270 && HUE > 210){
            return true;
        }
        else{
            return false;
        }
    }

    //raises and lowers jewel arm.
    public void lowerJewel() {
        jewelArm.setPosition(0.12);
    }

    public void raiseJewel() {
        jewelArm.setPosition(0.50);
    }
}
