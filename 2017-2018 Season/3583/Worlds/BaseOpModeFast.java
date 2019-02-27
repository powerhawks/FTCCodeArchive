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

public abstract class BaseOpModeFast extends LinearOpMode {
    //Initializes driving, shooting, and collecting motors for Evangelion Unit-01

    //Initializes motors and servos for the New Bot
//    public DcMotor lFrontDrive;
    public DcMotor lBackDrive;
//    public DcMotor rFrontDrive;
    public DcMotor rBackDrive;
    public DcMotor gLift;
    public Servo lGClamp;
    public Servo rGClamp;
    public Servo sensorArm;
    public ColorSensor sightBud;
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    float hsvValues[] = {0F,0F,0F};


    // inch movement methods
    //wheel radius
    static double WRadius = 1.9375;
    static double WCirc = (2 * 3.141592) * WRadius;
    // input:output, GR > 1 for torque, GR < 1 for speed
    static double GearRatio = 1;

    //////////////////////////////////////////// POSSIBLE ERROR: CHANGE TicksPerRev
    static int TicksPerRevolution = 560;
                                    // NEVEREST 20: 560
                                    //NEVEREST 40: 1120
                                    //NEVEREST 60: 1680

    // robot radius from turn point to wheels
    static double RRadius = 9.50;//7, 7.25, 8.25;
    static double RCirc = (2 * 3.141592) * RRadius;
    static int tolerance = 10;

    static double MOTOR_POWER = .75;
    static double KNOCK_MOTOR_POWER = .40;
    int ticksRight = 0;
    int ticksLeft = 0;


    // vuforia
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;


    public void initRobot(){
        // motors
        lBackDrive = hardwareMap.dcMotor.get("lBackDrive");
        lBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rBackDrive = hardwareMap.dcMotor.get("rBackDrive");
        rBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gLift = hardwareMap.dcMotor.get("gLift");
        gLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gLift.setDirection(DcMotorSimple.Direction.REVERSE);
        gLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // servos
        lGClamp = hardwareMap.servo.get("lGClamp");
        rGClamp = hardwareMap.servo.get("rGClamp");
        sensorArm = hardwareMap.servo.get("jewelArm");
        // sensors
        //sightBud = hardwareMap.colorSensor.get("sightBud");
        sightBud = hardwareMap.get(LynxI2cColorRangeSensor.class ,"sightBud");

        lBackDrive.setDirection(DcMotorSimple.Direction .REVERSE);
        lBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        this.relicTrackables.activate();

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
        lGClamp.setPosition(0.30);
        rGClamp.setPosition(.61);
    }

    //accepts negatives
    void LeftAdvance(double ticks){
        int init = lBackDrive.getCurrentPosition();

        if(ticks >= 0){
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


    void RightAdvance(double ticks){
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

    void logData() {
        telemetry.addData("lBack", lBackDrive.getCurrentPosition());
        telemetry.addData("rBack", rBackDrive.getCurrentPosition());
        telemetry.addData("lift", gLift.getCurrentPosition());
        telemetry.addData("vuMark", vuMark.toString());

        telemetry.update();
    }

    void forwardPos(int ticks){
        int targetTicksLeftBck = lBackDrive.getCurrentPosition() + ticks;
        int targetTicksRightBck = rBackDrive.getCurrentPosition() + ticks;

        lBackDrive.setTargetPosition(targetTicksLeftBck);
        rBackDrive.setTargetPosition(targetTicksRightBck);
        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER);

        try {
            boolean done = false;
            while (!done) {
                logData();
                if (!lBackDrive.isBusy() || !rBackDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            stopMotors();
        }
    }
    void backwardPos(int ticks) throws InterruptedException{
        int targetTicksLeftBck = lBackDrive.getCurrentPosition() - ticks;
        int targetTicksRightBck = rBackDrive.getCurrentPosition() - ticks;

        lBackDrive.setTargetPosition((targetTicksLeftBck));
        rBackDrive.setTargetPosition((targetTicksRightBck));
        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER);

        try {
            boolean done = false;
            while (!done) {
                logData();
                // if we are within 10 ticks or motors are no longer busy break
                // out of loop
                if (!lBackDrive.isBusy() || !rBackDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            stopMotors();
        }

    }

    // Run to Position:
    void turnLeftPos(int ticks) {
        try {

            lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() - ticks);
            rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() + ticks);
            rBackDrive.setPower(MOTOR_POWER);
            lBackDrive.setPower(MOTOR_POWER);
            waitForMotors(2000);
        } finally {
            stopMotors();
        }
    }

    void turnRightPos(int ticks) {
        try {
            lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() + ticks);
            rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() - ticks);
            rBackDrive.setPower(MOTOR_POWER);
            lBackDrive.setPower(MOTOR_POWER);
            waitForMotors(2000);
        } finally {
            stopMotors();
        }
    }

    void turnRight(double ticks){
        //use:All
        //reference: two motors?

        // negative right, positive left
        ticksRight -= ticks;
        ticksLeft += ticks;

        int init2 = rBackDrive.getCurrentPosition();
        int init1 = lBackDrive.getCurrentPosition();

        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER * -1.0);

        while( (lBackDrive.getCurrentPosition() - init1) < ticks ||
                (rBackDrive.getCurrentPosition() - init2) * -1 < ticks){
            // possible edit: remove * -1
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

        lBackDrive.setPower(MOTOR_POWER * -1.0);
        rBackDrive.setPower(MOTOR_POWER);

        while( (rBackDrive.getCurrentPosition() - init2) < ticks ||
                (lBackDrive.getCurrentPosition() - init1) * -1 < ticks) {
            // possible edit: remove * -1
        }
        stopMotors();
    }

    // wait for motors to not be busy for a max time of timeOut
    void waitForMotors(long timeOut) {
        long startTime = System.currentTimeMillis();

        while (lBackDrive.isBusy() || rBackDrive.isBusy()) {
            if (System.currentTimeMillis() - startTime > timeOut) {
                break;
            }
            idle();
        }
    }

    void raiseGlyph()  {
        gLift.setTargetPosition(gLift.getCurrentPosition() + 1000);
        gLift.setPower(1);

    }
    void lowerGlyph()  {
        gLift.setTargetPosition(gLift.getCurrentPosition() - 1000);
        gLift.setPower(1);
        waitForMotors(1000);
    }

    void stopMotors(){
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
    void pivotRightDeg(double deg){
        double inches = (deg / 360) * RCirc * 2;
        int ticks = (int) convertToTicks(inches);
        backwardPivotRight(ticks);
    }
    void pivotLeftDeg(double deg){
        double inches = (deg / 360) * RCirc * 2;
        int ticks = (int) convertToTicks(inches);
        backwardPivotLeft(ticks);
    }

    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800,
                (sightBud.blue() * 255) / 800, hsvValues);
    }

    //raises and lowers jewel arm.
    public void lowerJewel() {
        sensorArm.setPosition(0.12);
    }

    public void raiseJewel() {
        sensorArm.setPosition(0.50);
    }

    // Jewel knocking movements

    void knockLeft() throws InterruptedException {
        lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() - 50);
        rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() + 50);
        lBackDrive.setPower(KNOCK_MOTOR_POWER);
        rBackDrive.setPower(KNOCK_MOTOR_POWER);

        waitForMotors(1000);
    }

    void knockRight() throws InterruptedException {
        lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() + 50);
        rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() - 50);
        lBackDrive.setPower(KNOCK_MOTOR_POWER);
        rBackDrive.setPower(KNOCK_MOTOR_POWER);

        waitForMotors(1000);
    }

    void knockRightOld() throws InterruptedException {
        this.lBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER * -1.0);
        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            throw e;
        } finally {
            stopMotors();
            this.lBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.lBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.rBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }
    void wiggleLeft() throws InterruptedException {
        lGClamp.setPosition(.50);
        rGClamp.setPosition(.50);

        turnLeftDeg(20);
        forwardInch(5);
        backwardInch(1);
        openGlyphArm();
        forwardInch(3);
    }

    void wiggleRight() throws InterruptedException {
        lGClamp.setPosition(.50);
        rGClamp.setPosition(.50);

        turnRightDeg(20);
        forwardInch(5);
        backwardInch(1);
        openGlyphArm();
        forwardInch(3);
    }

    void wiggle() throws InterruptedException {
        lGClamp.setPosition(.50);
        rGClamp.setPosition(.50);

        turnLeftDeg(20);
        forwardInch(5);
        backwardInch(1);
        turnRightDeg(20);
        openGlyphArm();
        forwardInch(3);
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

    void determineVumarkPosition() {
        int maxTries = 10;
        while (maxTries>0) {
            sleep(500);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (RelicRecoveryVuMark.UNKNOWN == vuMark) {
                maxTries--;
                idle();
            } else {
                break;
            }
        }
    }

    void backwardPivotRight(int ticks) {
        lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition());
        rBackDrive.setTargetPosition(rBackDrive.getCurrentPosition() - ticks);
        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER);
        waitForMotors(5000);
    }

    void backwardPivotLeft(int ticks) {
        rBackDrive.setTargetPosition(lBackDrive.getCurrentPosition());
        lBackDrive.setTargetPosition(lBackDrive.getCurrentPosition() - ticks);
        lBackDrive.setPower(MOTOR_POWER);
        rBackDrive.setPower(MOTOR_POWER);
        waitForMotors(5000);
    }
}
