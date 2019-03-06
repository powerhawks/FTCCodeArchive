package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by Power Hawks Robotics on 11/5/2018.
 * Drive motor code by Zachary Young
 */

public abstract class BaseOpMode extends LinearOpMode {
    //Initializes driving, shooting, and collecting motors for Evangelion Unit-01

//    public DcMotor lDrive;
//    public DcMotor rDrive;
//    public DcMotor shooter;
//    public DcMotor collect;


    //Initializes motors and servos for World's Bot (Team 3583, 2018)
    public DcMotor RDrive;
    public DcMotor LDrive;
    public DcMotor liftRight;
    public DcMotor liftLeft;
//    public DcMotor CollectorArm;
    //public ColorSensor sightBud;
    //public Servo Sampler;
    public Servo Parker;
    public Servo markerGate;


    //    public Servo collectionRotate;
    public Servo Gate;
    BNO055IMU imu = null;
    Orientation angles;
    double globalAngle;

    float hsvValues[] = {0F,0F,0F};

    //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //CHANGE THIS VALUE key                                                                                                               //
    //------------------------------------------------------------------------------------------------------------------------------------//
    // 1. The diameter of your drive wheels /2 (for a 4" drive wheel enter 2)                                                             //
    // 2. Find the ticks per revolution value on the motor ((60:1680, 40:1120 20:560) or Neverest motors)                                 //
    // 3. RRadius the radius of the circle the drive wheels travel in.                                                                    //
    //      FOR 2 DRIVE WHEELS: Half the distance between wheels in inches                                                                //
    //      For 4 OR MORE DRIVE WHEELS: take the distance between the front right wheel and back left wheel in inches and half that value //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // inch movement methods
    //wheel radius
    static double WRadius = 1.9375; // CHANGE THIS VALUE 1
    static double WCirc = (2 * 3.141592) * WRadius;
    // input:output, GR > 1 for torque, GR < 1 for speed
    static double GearRatio = 1;

    static int TicksPerRevolution = 1120; // CHANGE THIS VALUE 2

    // robot radius from turn point to wheels
    static double RRadius = 8.25; // CHANGE THIS VALUE 3
    static double RCirc = (2 * 3.141592) * RRadius;
    static int tolerance = 10;

    int ticksRight = 0;
    int ticksLeft = 0;


    // vuforia
    /*
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private static final String VUFORIA_KEY = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
///////////////////////////////IMPORTANT///////////////////////////////////////////////////////////////////////////
///////////////////////ajust these values If using Vuforia//////////////////////////////////////////////////////////
    // Sample Values
    //final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    //final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    //final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    //Worlds Robit configuration

    //NOTE: Use front(audience) image when calibrating to not mix x and y
    final int CAMERA_FORWARD_DISPLACEMENT  = 25;//x coordnate
    final int CAMERA_VERTICAL_DISPLACEMENT = 295;//Z coordnate
    final int CAMERA_LEFT_DISPLACEMENT     = -41;  //Y coordnate

    final int CAMERA_HEADING = 114;
*/
    public void initRobot2(){
        // motors
        RDrive = hardwareMap.dcMotor.get("RDrive");
        LDrive = hardwareMap.dcMotor.get("LDrive");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        Parker = hardwareMap.servo.get("park");
        //Sampler = hardwareMap.servo.get("sample");
        markerGate = hardwareMap.servo.get("markerGate");


        //sightBud = hardwareMap.colorSensor.get("sightBud2");
//        collectionRotate = hardwareMap.servo.get("rotate");
//        CollectorArm = hardwareMap.dcMotor.get("collectorArm");
        Gate = hardwareMap.servo.get("gate");




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // servos
        // sampler = hardwareMap.servo.get("arm");
        // sensors
        //sightBud = hardwareMap.colorSensor.get("sightBud");
        //sightBud = hardwareMap.get(LynxI2cColorRangeSensor.class ,"sightBud");

        //Lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LDrive.setDirection(DcMotorSimple.Direction .REVERSE);

        RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void initRobot(){
        // motors
        RDrive = hardwareMap.dcMotor.get("RDrive");
        LDrive = hardwareMap.dcMotor.get("LDrive");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        Parker = hardwareMap.servo.get("park");
        //Sampler = hardwareMap.servo.get("sample");
        markerGate = hardwareMap.servo.get("markerGate");

        //Sampler = hardwareMap.servo.get("sample");

        //sightBud = hardwareMap.colorSensor.get("sightBud");
//        collectionRotate = hardwareMap.servo.get("rotate");
//        CollectorArm = hardwareMap.dcMotor.get("collectorArm");
//        Gate = hardwareMap.servo.get("gate");
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // servos
            // sampler = hardwareMap.servo.get("arm");
        // sensors
        //sightBud = hardwareMap.colorSensor.get("sightBud");
        //sightBud = hardwareMap.get(LynxI2cColorRangeSensor.class ,"sightBud");

        //Lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LDrive.setDirection(DcMotorSimple.Direction .REVERSE);

        RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        lFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initVuforia();
    }

/*
    public void initVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);



        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;


        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


//////////////////////////////////////////////////////
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        //roll, pitch, heading
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, CAMERA_HEADING));
/////////////////////////////////////////////////////
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
/*
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
/*
        targetsRoverRuckus.activate();
        while (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

    }*/


    public void sleep(int milli){
        try{
            Thread.sleep(milli);
        }
        catch(Exception e){

        }
    }


    /*
    public void ShowVufTelemetry(){
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle + 180);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }
    */

    /*
    public String getTarget(){
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                return trackable.getName();
            }
        }
        return "none";



    }
    */
    public void useEncoders(){
        LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void turnLeftGyro(double speed, double degrees){
        resetAngle();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double start = angles.firstAngle;
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
                LDrive.setPower(-speed);
                RDrive.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }else if(target > angles.firstAngle) {
            while (target > angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                LDrive.setPower(-speed);
                RDrive.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        StopMotors();
    }

    void turnRightGyro(double speed, double degrees) throws InterruptedException{
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
                LDrive.setPower(speed);
                RDrive.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }else if(target > angles.firstAngle) {
            while (target > angles.firstAngle) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                LDrive.setPower(speed);
                RDrive.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        StopMotors();
    }

    void resetAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }






    void logMotors() {
        telemetry.addData("LDrive", LDrive.getCurrentPosition());
        telemetry.addData("RDrive", RDrive.getCurrentPosition());

        telemetry.addData("Lifter", liftRight.getCurrentPosition());

        telemetry.update();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////Vuforia Drive Methods///////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    void vuDriveToPos(double targX,double targY,double targHeading){

        VectorF translation = lastLocation.getTranslation();
        double curX = translation.get(0)/ mmPerInch;
        double curY = translation.get(1)/ mmPerInch;
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        double curHeading = rotation.thirdAngle + 180;
        // head to position


        double moveX = 0;
        double moveY = 0;
        if (targX > curX){
            moveX = targX - curX;
        }
        else{
            moveX = curX - targX;
        }
        if (targY > curY){
            moveY = targY - curY;
        }
        else{
            moveY = curY - targY;
        }

        double degLeft = atan(moveY/moveX);

        if (degLeft > 0){
            turnLeftDeg(degLeft);
        }
        else if (degLeft < 0){
            turnRightDeg(-1 * degLeft);
        }



        // drive to pos
        VectorF translation2 = lastLocation.getTranslation();
        double curX2 = translation.get(0)/mmPerInch;
        double curY2 = translation.get(1)/mmPerInch;

        double moveX2 = 0;
        double moveY2 = 0;
        if (targX > curX2){
            moveX2 = targX - curX2;
        }
        else{
            moveX2 = curX2 - targX;
        }


        if (targY > curY2){
            moveY2 = targY - curY2;
        }
        else{
            moveY2 = curY2 - targY;
        }


        forwardInch((moveX2 * moveX2) + (moveY2 * moveY2));

        //turn to heading

        Orientation rotation2 = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        double curHeading2 = rotation.thirdAngle + 180;

        if (targHeading > curHeading2){
            turnLeftDeg(targHeading - curHeading2);
        }
        else if(targHeading < curHeading2){
            turnRightDeg(curHeading2 - targHeading);
        }
        else;



    }*/




    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////ENCODER DRIVE METHODS////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////Tick Methods (Position) //////////////////////////////

    // replicate these for two motors
    //////////////////////////Two Motors////////////////////////////////////////////////////////////

    void forwardPos(int ticks){
        int targetLDrive = LDrive.getCurrentPosition() + ticks;
        int targetRDrive = RDrive.getCurrentPosition() + ticks;

        LDrive.setTargetPosition(targetLDrive);
        RDrive.setTargetPosition(targetRDrive);
        LDrive.setPower(.75);
        RDrive.setPower(.75);

        try {
            boolean done = false;
            while (!done) {
                logMotors();
                if (!LDrive.isBusy() || !RDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            StopMotors();
        }
    }
    void backwardPos(int ticks) throws InterruptedException{
        int targetTicksLeftFrnt = LDrive.getCurrentPosition() - ticks;
        int targetTicksRightFrnt = RDrive.getCurrentPosition() - ticks;


        LDrive.setTargetPosition(targetTicksLeftFrnt);
        RDrive.setTargetPosition(targetTicksRightFrnt);

        LDrive.setPower(-.75);
        RDrive.setPower(-.75);


        try {
            boolean done = false;
            while (!done) {
                logMotors();
                // if we are within 10 ticks or motors are no longer busy then break
                // out of loop
                if (!LDrive.isBusy() || !RDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            StopMotors();
        }

    }

    void turnLeftPos(int ticks) {
        try {

            LDrive.setTargetPosition(LDrive.getCurrentPosition() - ticks);
            RDrive.setTargetPosition(RDrive.getCurrentPosition() + ticks);
            RDrive.setPower(.5);
            LDrive.setPower(-.5);

            waitForMotors(2000);
        } finally {
            StopMotors();
        }
    }

    void turnRightPos(int ticks) {
        try {
            LDrive.setTargetPosition(LDrive.getCurrentPosition() + ticks);
            RDrive.setTargetPosition(RDrive.getCurrentPosition() - ticks);

            RDrive.setPower(-.5);
            LDrive.setPower(.5);

            waitForMotors(2000);
        } finally {
            StopMotors();
        }
    }

    /*
    //////////////////////////Four Motors///////////////////////////////////////////////////////////
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
                // if we are within 10 ticks or motors are no longer busy then break
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
    */

    ////////////////////////////////////Tick Methods (Position) //////////////////////////////
    void turnRight(double ticks){
        //use:All
        //reference: two motors?

        // negative right, positive left
        ticksRight -= ticks;
        ticksLeft += ticks;

        int init2 = RDrive.getCurrentPosition();
        int init1 = LDrive.getCurrentPosition();

        LDrive.setPower(.75);
        RDrive.setPower(-.75);
        // for 4 motors, match left motors and match right motors charge, no encder needed for 2nd left and right motors

        while( (LDrive.getCurrentPosition() - init1) < ticks ||
                (RDrive.getCurrentPosition() - init2) * -1 < ticks){
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

        int init2 = RDrive.getCurrentPosition();
        int init1 = LDrive.getCurrentPosition();

        LDrive.setPower(-.75);
        RDrive.setPower(.75);


        while( (RDrive.getCurrentPosition() - init2) < ticks ||
                (LDrive.getCurrentPosition() - init1) * -1 < ticks) {
            sleep(10);
            // possible edit: remove * -1
        }
        StopMotors();
    }


    void LeftAdvance(double ticks){
        int init = LDrive.getCurrentPosition();

        if(ticks >= 0){
            LDrive.setPower(0.5);
        }
        else if (ticks <= 0){
            LDrive.setPower(-0.5);
        }
        while(LDrive.getCurrentPosition() - init < ticks){
            sleep(10);
        }
        StopMotors();
    }


    void RightAdvance(double ticks) {
        int init = RDrive.getCurrentPosition();

        if (ticks >= 0) {
            RDrive.setPower(0.5);
        } else if (ticks <= 0) {
            RDrive.setPower(-0.5);

        }
        while (RDrive.getCurrentPosition() - init < ticks) {
            sleep(10);
        }
        StopMotors();
    }

    ///////////////////////////////////////////Helper methods///////////////////////////////////////

    // wait for motors to not be busy for a max time of timeOut
    void waitForMotors(long timeOut) {
        long startTime = System.currentTimeMillis();

        while (LDrive.isBusy() || RDrive.isBusy()) // ||
               // lBackDrive.isBusy() || rBackDrive.isBusy()) {
            if (System.currentTimeMillis() - startTime > timeOut) {
                break;
            }
            idle();
    }


    double convertToTicks(double inch){

        return (inch / WCirc) * TicksPerRevolution * GearRatio;
    }

    void StopMotors(){

    // all motors.setPower(0.0);
        LDrive.setPower(0.0);
        RDrive.setPower(0.0);
        liftRight.setPower(0.0);
        liftLeft.setPower(0.0);
    }
///////////////////////////////Inch-Degree Methods//////////////////////////////////////////////////
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
        double CircInches = (deg / 360) * RCirc;
        int ticks = (int) convertToTicks(CircInches);
        turnRightPos(ticks);
    }
    void turnLeftDeg(double deg){
        double CircInches = (deg / 360) * RCirc;
        int ticks = (int) convertToTicks(CircInches);
        turnLeftPos(ticks);
    }







    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// HUE methods ////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////







    /*
    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800,
                (sightBud.blue() * 255) / 800, hsvValues);
    }







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






    //// USE THIS!!!! (2018-2019)
    boolean isColor(){
        if (isGold() && Saturation() > .3 && Saturation() < .51){
            return true;
        }
        return false;
    }

    boolean isGold(){
        updateColorSensor();
        float HUE = hsvValues[0];

        if (HUE < 51 && HUE > 20){
            return true;

        }
        else{
            return false;
        }
    }

    double Saturation(){
        return hsvValues[1];
    }


    */
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////VUPHORIA////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
/*
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
*/


    //////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////Unused Functions///////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////Time-Based////////////////////////////////////

    void forwardTimeout(double ticks, double ms) {
        ticksRight += ticks;
        ticksLeft += ticks;

        int init = LDrive.getCurrentPosition();

        LDrive.setPower(0.4);
        RDrive.setPower(0.4);

        //lFrontDrive.setPower(0.4);
        //rFrontDrive.setPower(0.4);
        //lBackDrive.setPower(0.4);
        //rBackDrive.setPower(0.4);
//        while(lFrontDrive.getCurrentPosition() ) {

//        }
        StopMotors();
    }
    // Zach's methods
    // Status: Not Operational

    void forwardTime(int ms) throws InterruptedException {

        RDrive.setPower(1.0);
        LDrive.setPower(1.0);

        //lFrontDrive.setPower(1.0);
        //rFrontDrive.setPower(1.0);
        //lBackDrive.setPower(1.0);
        //rBackDrive.setPower(1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    void backwardTime(int ms) throws InterruptedException {

        RDrive.setPower(-1.0);
        LDrive.setPower(-1.0);

        //lFrontDrive.setPower(-1.0);
        //rFrontDrive.setPower(-1.0);
        //lBackDrive.setPower(-1.0);
        //rBackDrive.setPower(-1.0);
        try {
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }


    void turnRightTime(int ms) throws InterruptedException {
        try {

            LDrive.setPower(1.0);
            RDrive.setPower(-1.0);

            //lFrontDrive.setPower(1);
            //lBackDrive.setPower(1);
            //rFrontDrive.setPower(-1);
            //rBackDrive.setPower(1);
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
        }
    void turnLeftTime(int ms) throws InterruptedException {
        try {

            LDrive.setPower(-1.0);
            RDrive.setPower(1.0);

            //lFrontDrive.setPower(-1);
            //lBackDrive.setPower(-1);
            //rFrontDrive.setPower(1);
            //rBackDrive.setPower(1);
            Thread.sleep(ms);
        } finally {
            StopMotors();
        }
    }
    ////////////////////////////Encoder Position correcting method (untested)///////////////////////
    // tick methods
    void fixPos() throws InterruptedException {
        int rightDiff = ticksRight - RDrive.getCurrentPosition() ;
        int leftDiff = ticksLeft - LDrive.getCurrentPosition() ;
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
    /////////////////////////////////////UNKNOWN////////////////////////////////////////////////////

    //    void blueOne() {
//        backwardPos(3);
//        TurnRight(2.2);
//    }
    //Thoughts on gyro use
//    void GyroRIght() {
//        int Orient =
//    }
    //accepts negatives


    //



////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////Relic Recovery Functions///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Programming Captain: Elijah Jenson
    //Team: Zachary Young and (Error 404: Teamate Fot Found)

    /*

    ////////////////////////////////////// Jewel knocking movements ////////////////////////////////////
    public void closeGlyphArm() {
        lGClamp.setPosition(0.55);
        rGClamp.setPosition(0.40);
    }
    public void openGlyphArm() {
        lGClamp.setPosition(0.22);
        rGClamp.setPosition(.69);
    }

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

/////////////////////////////////Glyph Methods (linear slide)///////////////////////////////////////////////

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

///////////////////////////////raises and lowers jewel arm.///////////////////////////////////////////////
    public void lowerJewel() {
        sensorArm.setPosition(0.42);
    }

    public void raiseJewel() {
        sensorArm.setPosition(0.0);
    }

    */



    ////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////Rover Ruckus//////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //5178 Programming Captain: Zachary Young
    //Programming Team: Jay Bridgman and Nate (Error 404: Last Name Not Found)

    //Sampler
    /*
    public boolean cubeSpotted (){
        if (isGold() && Saturation() < 70 ){
            return true;
        }
        return false;
    }
    */

    //Lower
    public void RobotDownInch ( double inch){
        double TPI = ((6.5 * 1680) / 3);//gear ratio times ticks per revolution

        RobotDownTicks((int) (TPI * inch));
    }
    public void RobotDownTicks ( int ticks){
        int targetH = liftLeft.getCurrentPosition() - ticks;
        liftLeft.setTargetPosition(targetH);
        //int previousPosition = 0;
        try {
            boolean done = false;
            while (!done) {
                liftLeft.setPower(-1.0);
                liftRight.setPower(-1.0);
                logMotors();
                if (!liftLeft.isBusy()) {
                    done = true;
                }
                if ((liftLeft.getCurrentPosition() - targetH > -50) && (liftLeft.getCurrentPosition() - targetH < 50)){
                    done = true;
                }
                telemetry.addData("Ticks moved in loop", liftLeft.getCurrentPosition() - targetH);
                updateTelemetry(telemetry);

                idle();
            }
        } finally {
            liftRight.setPower(0.0);
            liftLeft.setPower(0.0);
        }
    }
    /*
    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800,
                (sightBud.blue() * 255) / 800, hsvValues);
    }


    boolean isColor(){
        if (isGold() && Saturation() > .2 && Saturation() < .51){
            return true;
        }
        return false;
    }

    double Saturation(){
        return hsvValues[1];
    }

    boolean isGold(){
        updateColorSensor();
        float HUE = hsvValues[0];

        if (HUE < 65 && HUE > 20){
            return true;

        }
        else{
            return false;
        }
    }

*/

    /*

    public void Sample(){
        boolean SeeColor = false;
        boolean armDown = false;

        //Scanning
        for (int i = 1; i <= 3; i++ ){
            if(isColor()){
                SeeColor = true;
            }
            forwardInch(1 / 1.66);//moves 1 inch

        }

        //Deciding
        if (SeeColor){
            //put down arm    Tested value: .225
            Sampler.setPosition(.3);
            sleep(500);
            armDown = true;
        }
        //Knock or pass object
        forwardInch((12 / 3.66) * (1.2)); //moves 6 inch
        //
        if (armDown){
            //put arm up
            Sampler.setPosition(.15);
            armDown = false;

        }

    }
    */

    public void ParkUp(){
        Parker.setPosition(0);
    }
    public void ParkDown(){
        Parker.setPosition(.7);
    }
    public void OpenGate(){
        markerGate.setPosition(1);

    }
    public void CloseGate(){
        markerGate.setPosition(0);

    }



//    public void HoldMarker(){
//        collectionRotate.setPosition(1);
//        Gate.setPosition(.65);
//    }
//    public void DeployMarker(){
//        ExtendBucket(500);
//        collectionRotate.setPosition(.05);
//        Gate.setPosition(.5);
//        sleep(1000);
//    }
//
//    public void ExtendBucket(int millis){
//        CollectorArm.setPower(1);
//        sleep(millis);
//        CollectorArm.setPower(0);
//    }
//
//    public void DropBucket(){
//        collectionRotate.setPosition(1);
//        sleep(200);
//        collectionRotate.setPosition(.5);
//    }


}