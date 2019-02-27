package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * Created by Power Hawks Robotics on 12/5/2017.
 */

public abstract class BaseOpModeGyro extends LinearOpMode {
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
    /*
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);*/

    //Constants for gyro drive -- coppied over from the example class
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // robot radius from turn point to wheels
    static double RRadius = 9.50;//7, 7.25, 8.25;
    static double RCirc = (2 * Math.PI) * RRadius;
    static int tolerance = 10;

    static double MOTOR_POWER = .30;
    static double KNOCK_MOTOR_POWER = .40;
    int ticksRight = 0;
    int ticksLeft = 0;


    // vuforia
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    //init Gyro

    BNO055IMU imu;
    Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    Acceleration gravity  = imu.getGravity();

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

        //Init Gyro
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

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
    }

    void sleep(int milli){
        try{
            Thread.sleep(milli);
        }
        catch(Exception e){

        }

    }
    //Gryo

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(convertToTicks(distance));
            newLeftTarget = lBackDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            lBackDrive.setTargetPosition(newLeftTarget);
            rBackDrive.setTargetPosition(newRightTarget);

            lBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            lBackDrive.setPower(speed);
            rBackDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (lBackDrive.isBusy() && rBackDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                lBackDrive.setPower(leftSpeed);
                rBackDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      lBackDrive.getCurrentPosition(),
                        rBackDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            lBackDrive.setPower(0);
            rBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            lBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void gyroTurn(double speed, double angle){

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        lBackDrive.setPower(leftSpeed);
        rBackDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
        StopMotors();
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
        StopMotors();
    }

    //
    void forwardTimeout(double ticks, double ms) {
        ticksRight += ticks;
        ticksLeft += ticks;

        int init = lBackDrive.getCurrentPosition();
        lBackDrive.setPower(0.4);
        rBackDrive.setPower(0.4);
        StopMotors();
    }
    // Zach's methods
    // Status: Not Operational

    void forwardTime(int ms) throws InterruptedException {
        lBackDrive.setPower(1.0);
        rBackDrive.setPower(1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void backwardTime(int ms) throws InterruptedException {
        lBackDrive.setPower(-1.0);
        rBackDrive.setPower(-1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void logData() {
        telemetry.addData("lBack", lBackDrive.getCurrentPosition());
        telemetry.addData("rBack", rBackDrive.getCurrentPosition());
        telemetry.addData("lift", gLift.getCurrentPosition());

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
            StopMotors();
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
            StopMotors();
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
            StopMotors();
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
            StopMotors();
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
        StopMotors();
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
            sleep(10);
            // possible edit: remove * -1
        }
        StopMotors();
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

    void StopMotors(){
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
        sensorArm.setPosition(0.13);
    }

    public void raiseJewel() {
        sensorArm.setPosition(0.60);
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
            StopMotors();
            this.lBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.lBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.rBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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

    void turnRightTime(int ms) throws InterruptedException {
        try {
            lBackDrive.setPower(1);
            rBackDrive.setPower(1);
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void turnLeftTime(int ms) throws InterruptedException {
        try {
            lBackDrive.setPower(-1);
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
        int maxTries = 3;
        while (maxTries>0) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(500);
            if (RelicRecoveryVuMark.UNKNOWN == vuMark) {
                maxTries--;
                idle();
            } else {
                break;
            }
        }

        return vuMark;
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
