package org.firstinspires.ftc.teamcode.Rover_Ruckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class ThotimusSkeltonAutov10 extends TensorFlowSkelton {

    public DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, scoopLift, liftMechanism, collection;
    public Servo parkBar;
    //Gyro stuff
    BNO055IMU imu = null;
    Orientation angles;
    double globalAngle;
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    public VuforiaLocalizer vuforia;
    public VuforiaLocalizer.Parameters parameters;
    public int cameraMonitorViewId;

    //Lift positions
    public static final int LIFTUP = 15200; //max is 15232ish

    //Mecanum Drive
    //    4 motorS
    private static final String LEFT1NAME = "l1";
    private static final String LEFT2NAME = "l2";
    private static final String RIGHT1NAME = "r1";
    private static final String RIGHT2NAME = "r2";

    //Lifting Mechanism
    //      1 motor
    private static final String LIFTNAME = "lift";

    //Intake
    //    1 motor
    //    1 crservo
    private static final String SCOOPLIFT = "sLift";
    private static final String COLLECT = "collect";

    //Nevrest 20s have 560 TPI, 40 uses 1120, 60 uses 1680
    public static final double WHEELRADIUS = 2;
    public static final double WHEELCIRC = (2* 3.141592) * WHEELRADIUS;
    public static final double TICKSPERREVOLUTION = 560;
    public static final double GEARRATIO = 1;

    public static final long TICKSPERINCH = (long) 45.6456399103;
    public static final long INCHESPERTICK = (long) (1/44.5633840657);

    public static final long TICKSTURN1DEGREE = (2000/90);

    //field dimensions in inches
    public static final long INCHESBETWEENMINERALS = (long) 14.5;
    public static final long CENTEROFFIELDTOLANDERLEG = (long) 34.5;
    public static final long CENTEROFFIELDTOBOLTONANCHOR = (long) 59.4; //I don't know what this means, but it's given and might be useful
    public static final long WIDTHOFTILE = (long) 24;


    //Vuforia 6
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private static final String VUFORIA_KEY = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n";
    /**Get Vuforia Key**/ public static String getVuKey(){return VUFORIA_KEY;}
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    RelicRecoveryVuMark determineVumarkPosition() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        int maxTries = 1;
        while (maxTries>0) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (RelicRecoveryVuMark.UNKNOWN == vuMark) {
                maxTries--;
            }
            else{
                break;
            }
        }

        return vuMark;
    }

    double convertToTicks(double inch){
        return (inch/ WHEELCIRC) * TICKSPERREVOLUTION * GEARRATIO;
    }



    //this method is the base drive and mathematics for our mecanum algorithmic drive
    private static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        x = -x;
        c = -c;

        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;

        // try setting these equal to their front most counterpart
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        //double scaledPower = gearShift[gear];
        double scaledPower = 1;
        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower)); //sets to 1
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower)); //sets to -1
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower)); //sets to -1
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower)); //sets to 1


    }
    //Emergency method to cut power to all components
    public static void stopAll(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, DcMotor lift, DcMotor iLift){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        lift.setPower(0);
        iLift.setPower(0);
    }

    void stopRobot(){
        leftBackWheel.setPower(0);
        leftFrontWheel.setPower(0);
        rightBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        liftMechanism.setPower(0);
        scoopLift.setPower(0);
        collection.setPower(0);
    }

    void goForward(double power, long tick) throws InterruptedException {
        arcadeMecanum(power,0,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorForward(leftFrontWheel, tick);
        stopRobot();
    }

    void goBackwards(double power, long tick) throws InterruptedException {
        //to go back, all motors spin backwards
        arcadeMecanum(-power,0,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorBackward(leftFrontWheel, tick);
        stopRobot();
    }

    void goLeft(double power, long tick) throws InterruptedException {
        //to go back, all motors spin backwards
        arcadeMecanum(0,-power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorBackward(leftFrontWheel, tick);
        stopRobot();
    }

    public void goRight(double power, long tick) throws InterruptedException {
        //to go back, all motors spin backwards
        arcadeMecanum(0,power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorForward(leftBackWheel, tick);
        stopRobot();

    }

    public void diagonalRightForward(double power, long tick) throws InterruptedException {
        //ro go diagonal to the right and forwards the
        //front left wheel and back right wheel move forward
        arcadeMecanum(-power,-power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorForward(rightFrontWheel,tick);
        stopRobot();

    }

    public void diagonalRightBackward(double power, long tick) throws InterruptedException {
        //to go diagonal to the right and backwards the
        //front left wheel and back right wheel move backwards
        arcadeMecanum(power,-power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorBackward(rightBackWheel,tick);
        stopRobot();

    }

    public void diagonalLeftForward(double power, long tick) throws InterruptedException {
        //to go diagonal to the left and forwards the
        //front right wheel and back left wheel move forward
        arcadeMecanum(-power,power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorForward(leftFrontWheel,tick);
        stopRobot();

    }

    void diagonalLeftBackwards(double power, long tick) throws InterruptedException {
        //to go diagonal to the left and backwards the
        //front right wheel and back left wheel move backwards
        arcadeMecanum(power,power,0,leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel);
        waitForMotorBackward(leftBackWheel,tick);
        stopRobot();
    }

    public void land() throws InterruptedException {
        liftMechanism.setPower(1);
        waitForMotorForwardTimeout(liftMechanism, LIFTUP, 8000);
        liftMechanism.setPower(0);
        goForward(1, TICKSPERINCH*9);
    }

    public void initRobot(){
        //Mechanum wheels
        leftFrontWheel = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.LEFT1NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.LEFT2NAME);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.RIGHT2NAME);

        //Lift mechanism
        liftMechanism = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.LIFTNAME);
        liftMechanism.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMechanism.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collection = hardwareMap.dcMotor.get(COLLECT);

        scoopLift = hardwareMap.dcMotor.get(ThotimusSkeltonAutov10.SCOOPLIFT);
        scoopLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parkBar = hardwareMap.servo.get("parkBar");
        parkBar.setPosition(1);

        //Vuforia stuff
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraModelViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = getVuKey();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void logMotors() {
        int lf, lb, rf, rb;
        lf = leftFrontWheel.getCurrentPosition();
        lb = leftBackWheel.getCurrentPosition();
        rf = rightFrontWheel.getCurrentPosition();
        rb = rightBackWheel.getCurrentPosition();
        telemetry.addData("Motors", "%d,%d,%d,%d", lf, lb, rf, rb);

    }
    public void setMotorMode(DcMotor.RunMode mode) {
        leftFrontWheel.setMode(mode);
        rightFrontWheel.setMode(mode);
        leftBackWheel.setMode(mode);
        rightBackWheel.setMode(mode);
        liftMechanism.setMode(mode);
    }

    public void dropMarker() throws InterruptedException{
        collection.setPower(1);
        Thread.sleep(1000);
        collection.setPower(0);
    }

    public void moveToCraterDepot() throws InterruptedException{
        goBackwards(1, (long) (TICKSPERINCH*WIDTHOFTILE*5.4));
        parkBar.setPosition(0.5);

    }
    //Hits the correct mineral, ending in the same place every time
    static final long INCHESGOFORWARD = (long)47;
    static final long INCHESGOBACKWARD = (long)47;
    static final long INCHESGOFORWARDDEPOT = 60;
    static final double TURNDEGREES = 22.5;
    static final double TURNDEGREES2 = 45;
    static final double SPEED = 0.75;
    static final long FORWARDTODEPOT = 30;


    public void sample(String goldPos) throws InterruptedException{
        //Robot has just landed and oriented to see minerals
        //By this time, we have an image of the sampling area
        //TensorSkelton analyzes this image and returns the location of the gold cube
        if (goldPos.equals("Left")) {
            turnLeftGyro(.5, TURNDEGREES);
            goForward(.5, TICKSPERINCH*INCHESGOFORWARD);
            goBackwards(.5, TICKSPERINCH*INCHESGOBACKWARD);
            turnRightGyro(.5, TURNDEGREES);
            scoopLift.setPower(-1);
            Thread.sleep(500);
            scoopLift.setPower(0);
        } else if (goldPos.equals("Right")) {
            turnRightGyro(.5, TURNDEGREES);
            goForward(.5, TICKSPERINCH*INCHESGOFORWARD);
            goBackwards(.5, TICKSPERINCH*INCHESGOBACKWARD);
            turnLeftGyro(.5, TURNDEGREES);
            scoopLift.setPower(-1);
            Thread.sleep(500);
            scoopLift.setPower(0);
        } else if (goldPos.equals("Center")) {
            goForward(.5, TICKSPERINCH * INCHESGOFORWARD);
            goBackwards(.5, TICKSPERINCH * INCHESGOBACKWARD);
            scoopLift.setPower(-1);
            Thread.sleep(500);
            scoopLift.setPower(0);
        }
    }

    public void sampleLeft() throws InterruptedException{
        turnLeftGyro(SPEED, TURNDEGREES);
        goForward(SPEED, TICKSPERINCH*INCHESGOFORWARDDEPOT);
        turnRightGyro(SPEED, TURNDEGREES2);
        goForward(SPEED, TICKSPERINCH*FORWARDTODEPOT);
        scoopLift.setPower(-1);
        Thread.sleep(500);
        scoopLift.setPower(0);
        turnRightGyro(1, 17);
    }

    public void sampleRight() throws InterruptedException{
        turnRightGyro(SPEED, TURNDEGREES);
        goForward(SPEED, TICKSPERINCH*INCHESGOFORWARDDEPOT);
        turnLeftGyro(SPEED, TURNDEGREES2+10);
        goForward(SPEED, (long) (TICKSPERINCH*FORWARDTODEPOT*2));
        scoopLift.setPower(-1);
        Thread.sleep(500);
        scoopLift.setPower(0);
        turnRightGyro(1, 70);

    }

    public void sampleCenter() throws InterruptedException{
        goForward(1, (long) ((TICKSPERINCH*WIDTHOFTILE*5)));
        scoopLift.setPower(-1);
        Thread.sleep(500);
        scoopLift.setPower(0);
        turnRightGyro(1, 30);
    }

    public void waitForMotorBackward(DcMotor motor, long ticks) throws InterruptedException{
        long targetPos = motor.getCurrentPosition() - ticks;
        while(motor.getCurrentPosition() > targetPos){
            Thread.sleep(10);
        }
    }

    public void waitForMotorForward(DcMotor motor, long ticks) throws InterruptedException{
        long targetPos = motor.getCurrentPosition() + ticks;
        while (motor.getCurrentPosition() < targetPos){
            Thread.sleep(10);
        }
    }

    public void waitForMotorForwardTimeout(DcMotor motor, long ticks, double time) throws InterruptedException{
        long startTime = System.currentTimeMillis();
        long targetPos = motor.getCurrentPosition() + ticks;
        while (motor.getCurrentPosition() < targetPos){
            Thread.sleep(10);
            if(System.currentTimeMillis() - startTime >= time){
                break;
            }
        }
    }

    public void useEncoders(){
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void turnLeftGyro(double speed, double degrees){
        resetAngle();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = degrees + angles.firstAngle;
        if(target > 180){
            target -= 360;
        }else if(target < -180){
            target += 360;
        }
        if(target < angles.firstAngle) {
            while (target < angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                leftFrontWheel.setPower(-speed);
                leftBackWheel.setPower(-speed);
                rightBackWheel.setPower(speed);
                rightFrontWheel.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }else if(target > angles.firstAngle) {
            while (target > angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                leftFrontWheel.setPower(-speed);
                leftBackWheel.setPower(-speed);
                rightBackWheel.setPower(speed);
                rightFrontWheel.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        stopRobot();
    }

    void turnRightGyro(double speed, double degrees) {
        resetAngle();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle - degrees;
        if(target > 180){
            target -= 360;
        }else if(target < -180){
            target += 360;
        }
        if(target < angles.firstAngle) {
            while (target < angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                leftFrontWheel.setPower(speed);
                leftBackWheel.setPower(speed);
                rightBackWheel.setPower(-speed);
                rightFrontWheel.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }else if(target > angles.firstAngle) {
            while (target > angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                leftFrontWheel.setPower(speed);
                leftBackWheel.setPower(speed);
                rightBackWheel.setPower(-speed);
                rightFrontWheel.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        stopRobot();
    }

    void resetAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newLeftTargetBack;
        int     newRightTarget;
        int     newRightTargetBack;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * TICKSPERINCH);
            newLeftTarget = leftFrontWheel.getCurrentPosition() + moveCounts;
            newRightTarget = rightFrontWheel.getCurrentPosition() + moveCounts;
            newLeftTargetBack = leftBackWheel.getCurrentPosition() + moveCounts;
            newRightTargetBack = rightBackWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFrontWheel.setTargetPosition(newLeftTarget);
            leftBackWheel.setTargetPosition(newLeftTargetBack);
            rightFrontWheel.setTargetPosition(newRightTarget);
            rightBackWheel.setTargetPosition(newRightTargetBack);

            leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            rightBackWheel.setPower(speed);
            rightFrontWheel.setPower(speed);
            leftBackWheel.setPower(speed);
            leftFrontWheel.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy())) {
                logMotors();
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

                leftFrontWheel.setPower(leftSpeed);
                leftBackWheel.setPower(leftSpeed);
                rightBackWheel.setPower(rightSpeed);
                rightFrontWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftFrontWheel.getCurrentPosition(),
                        rightFrontWheel.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftBackWheel.setPower(0);
            leftFrontWheel.setPower(0);
            rightFrontWheel.setPower(0);
            rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void lowerLift(){
        liftMechanism.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMechanism.setTargetPosition(100);
        liftMechanism.setPower(1);
    }

    public void moveToDepotDepot() throws InterruptedException{
        goForward(1, (long) ((TICKSPERINCH*WIDTHOFTILE*4.3)));
        turnRightGyro(1, 33);
    }

    public void moveToDepotCrater() throws InterruptedException{
        goForward(1, TICKSPERINCH*12);
        turnLeftGyro(1, 70);
        goForward(1, (long) (TICKSPERINCH*WIDTHOFTILE*4.0));
        turnLeftGyro(.5, 40);
        goForward(1, (long) (TICKSPERINCH*WIDTHOFTILE*2.8));
    }

    public void moveToCraterCrater() throws InterruptedException{
        goBackwards(1, (long) (TICKSPERINCH*WIDTHOFTILE*5.8));
        parkBar.setPosition(0.5);
    }

    public void dropMarkerDepot() throws InterruptedException{
        goForward(1,TICKSPERINCH*7);
        goBackwards(1,TICKSPERINCH*5);
        collection.setPower(1);
        Thread.sleep(1000);
        collection.setPower(0);

    }

}
