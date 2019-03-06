package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//Created by Team 3583 on 10/26/2017

public abstract class SkeletonOpMode extends LinearOpMode
{


    /*


// motors
    //Moves the robot
    public DcMotor lFrontDrive;
    public DcMotor lBackDrive;
    public DcMotor rFrontDrive;
    public DcMotor rBackDrive;
    //Raises the Glyph Lift
    public DcMotor gLift;
    //lifts and lowers the arm
    public DcMotor relicJoint;
    //Extends and retracts the relic arm
    public DcMotor relicArm;

//servos
    //Grabs the Relic
    public Servo relicClamp;
    //Grabs the Glyphs
    public Servo lGClamp;
    public Servo rGClamp;
    //Lowers and raises the side jewel arm
    public Servo jewelArm;

  //sensors
    public ColorSensor sightBud;
    float hsvValues[] = {0F,0F,0F};
    */

    //Motors
    public DcMotor RDrive;
    public DcMotor LDrive;
    public DcMotor liftRight;
    public DcMotor liftLeft;
    public Servo markerGate;
    public Servo Parker;
    //public DcMotor CollectorArm;
    //public DcMotor CollectionRaise;
    //public Servo collectionRotate;


    //public Servo Sampler;
    //public CRServo collectionWheel;
    //public Servo Gate;



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

        /*
        lFrontDrive = hardwareMap.dcMotor.get("lFrontDrive");
        rFrontDrive = hardwareMap.dcMotor.get("rFrontDrive");
        lBackDrive = hardwareMap.dcMotor.get("lBackDrive");
        rBackDrive = hardwareMap.dcMotor.get("rBackDrive");
        gLift = hardwareMap.dcMotor.get("gLift");


        relicJoint = hardwareMap.dcMotor.get("relicJoint");
        //  up/down
        //relicArm = hardwareMap.dcMotor.get("relicArm");
        //  in/out
        lGClamp = hardwareMap.servo.get("lGClamp");
        rGClamp = hardwareMap.servo.get("rGClamp");


//        relicClamp = hardwareMap.servo.get("relicClamp");

//Sets the motors to run without an encoder
        lFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */


        RDrive = hardwareMap.dcMotor.get("RDrive");
        LDrive = hardwareMap.dcMotor.get("LDrive");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        markerGate = hardwareMap.servo.get("markerGate");
        Parker = hardwareMap.servo.get("park");
        //Sampler = hardwareMap.servo.get("sample");
        //CollectorArm = hardwareMap.dcMotor.get("collectorArm");
        //collectionWheel = hardwareMap.crservo.get("collectionW");
        //CollectionRaise = hardwareMap.dcMotor.get("raise");
        //collectionRotate = hardwareMap.servo.get("rotate");
        //Gate = hardwareMap.servo.get("gate");


        //collectionR.setDirection(CRServo.Direction.REVERSE);

        //delete later?????
        /*
        RDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        */


        RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //collectionRotate.setPosition(.0);

    }


    void sleep(int milli){
        try{
            Thread.sleep(milli);
        }
        catch(Exception e){
        }
    }

    void stopMotors(){
        RDrive.setPower(0.0);
        LDrive.setPower(0.0);
    }

    public void CloseGate(){

        markerGate.setPosition(0);
    }

    public void OpenGate(){
        markerGate.setPosition(1);
    }
    public void ParkUp(){
        Parker.setPosition(0);
    }
    public void ParkDown(){
        Parker.setPosition(.7);
    }






    /*
    double convertToTicks(double inch){

        return (inch / WCirc) * TicksPerRevolution * GearRatio;
    }
    // tick methods
    void fixPos(){
        int rightDiff = ticksRight - rFrontDrive.getCurrentPosition() ;
        int leftDiff = ticksLeft - lFrontDrive.getCurrentPosition() ;

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
        int init = lFrontDrive.getCurrentPosition();

        if(ticks >= 0){
            lFrontDrive.setPower(0.5);
            lBackDrive.setPower(0.5);
        }
        else if (ticks <= 0){
            lFrontDrive.setPower(-0.5);
            lBackDrive.setPower(-0.5);
        }
        while(lFrontDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();
    }

    void rightAdvance(double ticks){
        int init = rFrontDrive.getCurrentPosition();

        if(ticks >= 0){
            rFrontDrive.setPower(0.5);
            rBackDrive.setPower(0.5);
        }
        else if (ticks <= 0){
            rFrontDrive.setPower(-0.5);
            rBackDrive.setPower(-0.5);
        }

        while(rFrontDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();
    }

    void forward(double ticks){
        //use: All
        //reference: one motor
        ticksRight += ticks;
        ticksLeft += ticks;

        int init = lFrontDrive.getCurrentPosition();
        lFrontDrive.setPower(1.0);
        rFrontDrive.setPower(1.0);
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        while(lFrontDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        stopMotors();

    }
    void backward(double ticks){
        //use:All
        //reference: one motor
        ticksRight -= ticks;
        ticksLeft -= ticks;

        int init = lFrontDrive.getCurrentPosition();
        lFrontDrive.setPower(-1.0);
        rFrontDrive.setPower(-1.0);
        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        while(lFrontDrive.getCurrentPosition() - init * -1 < ticks){
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

        int init2 = rFrontDrive.getCurrentPosition();
        int init1 = lFrontDrive.getCurrentPosition();

        lFrontDrive.setPower(1.0);
        rFrontDrive.setPower(-1.0);
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(-1.0);

        while((lFrontDrive.getCurrentPosition() - init1 < ticks) && (rFrontDrive.getCurrentPosition() - init2 * -1 < ticks)){
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

        int init2 = rFrontDrive.getCurrentPosition();
        int init1 = lFrontDrive.getCurrentPosition();

        lFrontDrive.setPower(-1.0);
        rFrontDrive.setPower(1.0);
        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(1.0);

        while((rFrontDrive.getCurrentPosition() - init2 < ticks) && (lFrontDrive.getCurrentPosition() - init1 * -1 < ticks)){
            sleep(10);                                                                       // possible edit: remove * -1
        }
        stopMotors();
    }

    void stopMotors(){
        lFrontDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
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
    */
}
