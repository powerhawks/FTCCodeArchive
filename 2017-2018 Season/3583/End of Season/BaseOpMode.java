package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Power Hawks Robotics on 12/5/2017.
 */

public abstract class BaseOpMode extends LinearOpMode {
    //Initializes driving, shooting, and collecting motors for Evangelion Unit-01

//    public DcMotor lDrive;
//    public DcMotor rDrive;
//    public DcMotor shooter;
//    public DcMotor collect;


    //Initializes motors and servos for the New Bot
    public DcMotor lFrontDrive;
    public DcMotor lBackDrive;
    public DcMotor rFrontDrive;
    public DcMotor rBackDrive;
    public DcMotor gLift;
    public Servo lGClamp;
    public Servo rGClamp;
    public Servo sensorArm;
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


    // vuforia
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;


    public void initRobot(){
        // motors
        lFrontDrive = hardwareMap.dcMotor.get("lFrontDrive");
        lBackDrive = hardwareMap.dcMotor.get("lBackDrive");
        rFrontDrive = hardwareMap.dcMotor.get("rFrontDrive");
        rBackDrive = hardwareMap.dcMotor.get("rBackDrive");
        gLift = hardwareMap.dcMotor.get("gLift");
        // servos
        lGClamp = hardwareMap.servo.get("lGClamp");
        rGClamp = hardwareMap.servo.get("rGClamp");
        sensorArm = hardwareMap.servo.get("arm");
        // sensors
        //sightBud = hardwareMap.colorSensor.get("sightBud");
        sightBud = hardwareMap.get(LynxI2cColorRangeSensor.class ,"sightBud");

        gLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFrontDrive.setDirection(DcMotorSimple.Direction .REVERSE);
        lBackDrive.setDirection(DcMotorSimple.Direction .REVERSE);
        lFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // init vuforia

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
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
    void fixPos() throws InterruptedException {
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
            forwardPos(rightDiff);
            // forwardPos
        }
        else if (RightSub && LeftSub){
            backwardPos((int)(rightDiff * -1));
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
            RightAdvance(rightDiff);
        }
        else if (LeftAdd || LeftSub){
            LeftAdvance(leftDiff);
        }
        else{

        }
    }
    public void closeGlyphArm() {
        lGClamp.setPosition(0.55);
        rGClamp.setPosition(0.40);
    }
    public void openGlyphArm() {
        lGClamp.setPosition(0.22);
        rGClamp.setPosition(.69);
    }
//    void blueOne() {
//        backwardPos(3);
//        TurnRight(2.2);
//    }
    //Thoughts on gyro use
//    void GyroRIght() {
//        int Orient =
//    }
    //accepts negatives
    void LeftAdvance(double ticks){
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
        StopMotors();
    }


    void RightAdvance(double ticks){
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
        StopMotors();
    }

    //
    void forwardTimeout(double ticks, double ms) {
        ticksRight += ticks;
        ticksLeft += ticks;

        int init = lFrontDrive.getCurrentPosition();
        lFrontDrive.setPower(0.4);
        rFrontDrive.setPower(0.4);
        lBackDrive.setPower(0.4);
        rBackDrive.setPower(0.4);
//        while(lFrontDrive.getCurrentPosition() ) {

//        }
        StopMotors();
    }
    // Zach's methods
    // Status: Not Operational

    void forwardTime(int ms) throws InterruptedException {
        lFrontDrive.setPower(1.0);
        rFrontDrive.setPower(1.0);
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void backwardTime(int ms) throws InterruptedException {
        lFrontDrive.setPower(-1.0);
        rFrontDrive.setPower(-1.0);
        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void logMotors() {
        telemetry.addData("lFront", lFrontDrive.getCurrentPosition());
        telemetry.addData("rFront", rFrontDrive.getCurrentPosition());
        telemetry.addData("lBack", lBackDrive.getCurrentPosition());
        telemetry.addData("rBack", rBackDrive.getCurrentPosition());

        telemetry.update();
    }
    void forwardPos(int ticks){
        int targetTicksLeftFrnt = lFrontDrive.getCurrentPosition() + ticks;
        int targetTicksRightFrnt = rFrontDrive.getCurrentPosition() + ticks;
        int targetTicksLeftBck = lBackDrive.getCurrentPosition() + ticks;
        int targetTicksRightBck = rBackDrive.getCurrentPosition() + ticks;

        lFrontDrive.setTargetPosition(targetTicksLeftFrnt);
        rFrontDrive.setTargetPosition(targetTicksRightFrnt);
        lBackDrive.setTargetPosition(targetTicksLeftBck);
        rBackDrive.setTargetPosition(targetTicksRightBck);
        lFrontDrive.setPower(1);
        rFrontDrive.setPower(1);
        lBackDrive.setPower(1);
        rBackDrive.setPower(1);

        try {
            boolean done = false;
            while (!done) {
                logMotors();
                if (!lFrontDrive.isBusy() || !rFrontDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            StopMotors();
        }
    }
    void backwardPos(int ticks) throws InterruptedException{
        int targetTicksLeftFrnt = lFrontDrive.getCurrentPosition() - ticks;
        int targetTicksRightFrnt = rFrontDrive.getCurrentPosition() - ticks;
        int targetTicksLeftBck = lBackDrive.getCurrentPosition() - ticks;
        int targetTicksRightBck = rBackDrive.getCurrentPosition() - ticks;

        lFrontDrive.setTargetPosition(targetTicksLeftFrnt);
        rFrontDrive.setTargetPosition(targetTicksRightFrnt);
        lBackDrive.setTargetPosition((targetTicksLeftBck));
        rBackDrive.setTargetPosition((targetTicksRightBck));
        lFrontDrive.setPower(1.0);
        rFrontDrive.setPower(1.0);
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(1.0);

        try {
            boolean done = false;
            while (!done) {
                logMotors();
                // if we are within 10 ticks or motors are no longer busy break
                // out of loop
                if (!lFrontDrive.isBusy() || !rFrontDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            StopMotors();
        }

    }

    // Run to Position:
    void turnLeftPos(int ticks) {
        try {

            lFrontDrive.setTargetPosition(lFrontDrive.getCurrentPosition() - ticks);
            rFrontDrive.setTargetPosition(rFrontDrive.getCurrentPosition() + ticks);
            lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() - ticks);
            rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() + ticks);
            rFrontDrive.setPower(1);
            lFrontDrive.setPower(1);
            rBackDrive.setPower(1);
            lBackDrive.setPower(1);
            waitForMotors(2000);
        } finally {
            StopMotors();
        }
    }

    void turnRightPos(int ticks) {
        try {
            lFrontDrive.setTargetPosition(lFrontDrive.getCurrentPosition() + ticks);
            rFrontDrive.setTargetPosition(rFrontDrive.getCurrentPosition() - ticks);
            lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() + ticks);
            rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() - ticks);
            rFrontDrive.setPower(1);
            lFrontDrive.setPower(1);
            rBackDrive.setPower(1);
            lBackDrive.setPower(1);
            waitForMotors(2000);
        } finally {
            StopMotors();
        }
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

        while( (lFrontDrive.getCurrentPosition() - init1) < ticks ||
                (rFrontDrive.getCurrentPosition() - init2) * -1 < ticks){
            // possible edit: remove * -1
        }
        StopMotors();
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

        while( (rFrontDrive.getCurrentPosition() - init2) < ticks ||
                (lFrontDrive.getCurrentPosition() - init1) * -1 < ticks) {
            sleep(10);
            // possible edit: remove * -1
        }
        StopMotors();
    }

    // wait for motors to not be busy for a max time of timeOut
    void waitForMotors(long timeOut) {
        long startTime = System.currentTimeMillis();

        while (lFrontDrive.isBusy() || rFrontDrive.isBusy() ||
                lBackDrive.isBusy() || rBackDrive.isBusy()) {
            if (System.currentTimeMillis() - startTime > timeOut) {
                break;
            }
            idle();
        }
    }

    void raiseGlyph() throws InterruptedException {
        gLift.setPower(1);
        try {
            Thread.sleep(500);
        } finally {
            gLift.setPower(0);
        }
    }
    void lowerGlyph() throws InterruptedException {
        gLift.setPower(-1);
        try {
            Thread.sleep(500);
        } finally {
            gLift.setPower(0);
        }
    }


    void StopMotors(){
        lFrontDrive.setPower(0.0);
        rFrontDrive.setPower(0.0);
        lBackDrive.setPower(0.0);
        rBackDrive.setPower(0.0);
        gLift.setPower(0.0);
    }

    // inch methods (More Advanced)
    void forwardInch(double inch){
        double ticks = convertToTicks(inch);
        forwardPos((int)ticks);
    }
    void backwardInch(double inch) throws InterruptedException{
        double ticks = convertToTicks(inch);
        backwardPos((int)ticks);
    }

    //Turning degrees?
    void turnRightDeg(double deg){
        double inches = (deg / 360) * RCirc;
        int ticks = (int) convertToTicks(inches);
        turnRightPos(ticks);
    }
    void turnLeftDeg(double deg){
        double inches = (deg / 360) * RCirc;
        int ticks = (int) convertToTicks(inches);
        turnLeftPos(ticks);
    }


    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800,
                (sightBud.blue() * 255) / 800, hsvValues);
    }

    //raises and lowers jewel arm.
    public void lowerJewel() {
        sensorArm.setPosition(0.42);
    }

    public void raiseJewel() {
        sensorArm.setPosition(0.0);
    }



    // Jewel knocking movements

    void knockRight() throws InterruptedException {
        this.lFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFrontDrive.setPower(.5);
        rFrontDrive.setPower(-.5);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw e;
        } finally {
            StopMotors();
            this.lFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
    void wiggle() throws InterruptedException {
        lGClamp.setPosition(.50);
        rGClamp.setPosition(0.50);

        turnLeftDeg(20);
        forwardInch(7);
        backwardInch(2);
        turnRightDeg(20);
        openGlyphArm();
        forwardInch(7);


    }

    void knockLeft() throws InterruptedException {
        this.lFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFrontDrive.setPower(-.5);
        rFrontDrive.setPower(.5);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw e;
        } finally {
            StopMotors();
            this.lFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    void turnRightTime(int ms) throws InterruptedException {
        try {
            lFrontDrive.setPower(1);
            lBackDrive.setPower(1);
            rFrontDrive.setPower(-1);
            rBackDrive.setPower(1);
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void turnLeftTime(int ms) throws InterruptedException {
        try {
            lFrontDrive.setPower(-1);
            lBackDrive.setPower(-1);
            rFrontDrive.setPower(1);
            rBackDrive.setPower(1);
            Thread.sleep(ms);
        } finally {
            StopMotors();
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

    RelicRecoveryVuMark determineVumarkPosition() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        int maxTries = 1;
        while (maxTries>0) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (RelicRecoveryVuMark.UNKNOWN == vuMark) {
                maxTries--;
                idle();
            } else {
                break;
            }
        }

        return vuMark;
    }
}
