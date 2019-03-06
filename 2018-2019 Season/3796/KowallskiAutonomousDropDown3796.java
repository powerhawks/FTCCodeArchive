package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enums.KowallskiSide3796;
import org.firstinspires.ftc.teamcode.Enums.MaterialOrder;
import org.firstinspires.ftc.teamcode.Enums.MotorSide;
import org.firstinspires.ftc.teamcode.HelperClasses.CubeGuesser;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiCollectionServos3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMineralArm3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiRamp3796;

import java.util.List;

/**
 *Made on December 9, 2018
 *Chase Galey and Lincoln Doney
 *A simple autonomous code to Let the Robot "land"
*/
@Autonomous(name="Qualifier Autonomous", group="Final")
@Disabled
public class KowallskiAutonomousDropDown3796 extends LinearOpMode {
    //Tensorflow models and naming stuff: (DO NOT TOUCH UNLESS YOU WANT TO BREAK THE ROBOT)
    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Abdg8LX/////AAABmSf7RINGpU/jkpjLj3pF/N0RGUB6ns3w7MLkNCju9HKm4f1tgfE3Ya/IjpsCrd9UXKIlUZPES9za475tuCfCA5gCsmyGJLf64mgG1jj375x/B6fOvV3aTyvTH7oOO8Nd9SR903r9LWcmwS37WxMZSKzJrtek2WBcoWOzTVEe/Cx7gnkmh8SjcEMIf2W3AToIzZ01SNmo5W39vHxJ6vlhmbndSvpdQBL81PZpBvZVH0Jz9qZTB1F2Efrs1rfQSTLILTQ7Y9fSNZUdTVZD/sulu86uorlL35IvLJtuTeBe+1hpb5/zHgVPHZH3saBmR01tPBfDk6Kjp41lYvdC8on7W3xvJCUdqT1AS/hmf7fv2N/G"
            + "\n";

    /** ALL OF THESE VALUES CAN AND SHOULD BE CHANGED.. THEY RUN THE AUTONOMOUS */

    //Power for the encoder-based driving (A CONSTANT)
    static final double AUTO_POWER = 0.25;
    //Width of robot (KEEP THIS CONSTANT UNLESS IT CHANGES)
    static final double ROBOT_WIDTH = 17;
    //Modifier for how much it goes left or right
    static final double L_R_MOD = 7;
    //Height of Robot (Not really used)
    static final double ROBOT_HEIGHT = 17;
    //Amount of ticks per inch on the left back wheel
    static final double TICKS_PER_INCH_L = (5122.0 / 120);
    //Amount of ticks per inch on the right back wheel
    static final double TICKS_PER_INCH_R = (5133.0 / 120);
    //Amount of power the encoders accelerate by per run
    static final double ACCELERATION_VALUE = 0.01;
    //Maximum power for the acceleration (Larger is faster)
    static final double ACCELERATION_MAX = 0.6;
    //Amount of time before the robot guesses the cube
    private static final int TIME_BEFORE_DEFAULT = 20 /*Times per second*/ * (4 /*Seconds*/);

    /** End of all of the constants */

    protected KowallskiSide3796 side = KowallskiSide3796.Depot;

    /**Intra-Method variables (MOST OF THEM ARE DEFINED IN THE setup() method*/
    //The webcam
    WebcamName webcamName;
    //The recognitions of the Tensorflow
    List<Recognition> updatedRecognitions = null;
    //The Imu object reference
    BNO055IMU imu = null;
    //The orientation of the robot (Here because it should be accessed everywhere
    Orientation angles;
    //Tensor Flow Object (FOR VISION)
    private TFObjectDetector tfod;
    //Vuforia Object (FOR VISION)
    private VuforiaLocalizer vuforia;
    //Unused but I dont want to get rid of it in case it messes something up
    private double globalAngle = 0;
    /**End of Intra-Method variables*/

    /** All of the Object variables*/
    //Wheels/Driving controller
    private KowallskiMecanumDrive3796 drive;
    //Ramp controller (Extender and lifter)
    private KowallskiRamp3796 ramp;
    //Arm controller
    private KowallskiMineralArm3796 arm;
    //Collection servo controller
    private KowallskiCollectionServos3796 servo;
    /** End of Object variables*/

    @Override
    public void runOpMode() throws InterruptedException {
        //Call the setup method (Initializes Intra-Method variables)
        setup();

        //Wait for the start of the program
        waitForStart();

        //Ensure the program only runs once
        Boolean firstTime = true;

        //Run program
        while (opModeIsActive()) {
            if (firstTime) {
                //Unlatch Lock
                telemetry.addLine("Initializing Autonomous...");
                telemetry.update();
                arm.moveUpOrDown(1);
                telemetry.addLine("Unlocking Arm...");
                telemetry.update();
                sleep(250);
                arm.stopMovement();

                //Lowering robot
                telemetry.addLine("Lowering down...");
                telemetry.update();
                arm.slowlyLower();
                sleep(1500);

                //Unlatch robot from hook
                telemetry.addLine("Unlatching from Hook...");
                telemetry.update();
                arm.stopMovement();
                sleep(1500);
                drive.stopMovement();
                sleep(100);

                //Move robot out of hook
                telemetry.addLine("Moving Left...");
                telemetry.update();
                DriveLeft((ROBOT_WIDTH - L_R_MOD - 1));
                //Move robot forward to ensure it does not re-attach
                telemetry.addLine("Moving Forward...");
                telemetry.update();
                DriveForward(5);

                //Move robot right to recenter it
                telemetry.addLine("Moving Right...");
                telemetry.update();
                DriveRight(ROBOT_WIDTH - L_R_MOD - 1);

                //Move arm back down
                telemetry.addLine("Moving arm down...");
                telemetry.update();
                arm.slowlyLower();
                sleep(75);
                arm.stopMovement();

                //Reposition robot in front of hook
                telemetry.addLine("Moving Backward...");
                telemetry.update();
                DriveBackward(3);

                /************************************************************
                 *Branches off to three periods based on gold position here:*
                 ************************************************************/
                //Check the order initially
                MaterialOrder materialOrder = checkorder();
                //Update telemetry based on results
                telemetry.update();
                //Set timer to zero
                int count = 0;
                //Loop until it sees something or timer ends
                while (opModeIsActive() && (materialOrder == MaterialOrder.Unknown && count <= TIME_BEFORE_DEFAULT)) {
                    //Check the material order
                    materialOrder = checkorder();
                    //Increment the count
                    count++;
                    //Sleep for 50 to not overload the CPU (Not sure if this is needed but its precautionary)
                    sleep(50);
                    //Update the telemetry
                    telemetry.update();
                }
                //If the material order isn't certian.. it takes a guess
                if(materialOrder == MaterialOrder.Unknown && updatedRecognitions != null)
                {
                    telemetry.addLine("Cube not 100%! Running guessing algorithm (Courtesy of Lincoln, the best programmer ever)");
                    telemetry.update();
                    //Runs the algorithm
                    materialOrder = CubeGuesser.guess(updatedRecognitions);
                    telemetry.addLine("Guess results: " + materialOrder.name());
                    telemetry.update();
                }
                //Robot is sure
                else
                {
                    telemetry.addLine("Cube 100% certian! Continuing as normal");
                    telemetry.addLine("Cube at: " + materialOrder.name());
                    telemetry.update();
                }
                //Branches the autonomous based on the cases
                branchAuto(materialOrder);
                //Stop the autonomous
                drive.stopMovement();
                firstTime = false;
            }

        }
    }

    void branchAuto(MaterialOrder m) throws InterruptedException {
        telemetry.addLine("Branching...");
        telemetry.update();
        switch (m) {
            case Left:
                //Material on Left
                telemetry.addLine("Gold on Left");
                telemetry.update();

                sleep(1000);

                //Move away from the lander
                DriveForward(4);

                if(side == KowallskiSide3796.Depot) {
                    telemetry.addLine("Moving Forward (Depot)...");
                    telemetry.update();
                    //Move in front of mineral
                    DriveLeft(ROBOT_WIDTH - L_R_MOD + 16);
                    //Course correction
//                    TurnLeft(2);
                    //Run over mineral
                    DriveForward(20);
                }else
                {
                    telemetry.addLine("Moving Forward (Crater)...");
                    telemetry.update();
                    //Move In front of Mineral
                    DriveLeft(ROBOT_WIDTH - L_R_MOD + 6);
                    //Run over mineral
                    DriveForward(13);
                }

                break;
            case Right:
                //Material on Right
                telemetry.addLine("Gold on Right");
                telemetry.update();

                sleep(1000);

                //Move away from lander
                DriveForward(4);

                if(side == KowallskiSide3796.Depot) {
                    //Go in front of mineral
                    DriveRight(ROBOT_WIDTH - L_R_MOD);
                    //Run over mineral
                    DriveForward(20);
                }else
                {
                    //Go in front of mineral
                    DriveRight(ROBOT_WIDTH - L_R_MOD + 6);
                    //Run over mineral
                    DriveForward(13);
                }

                break;
            default:
                //Material in the Center or Not Found
                telemetry.addLine("Gold Pos: " + m);
                telemetry.update();

                sleep(1000);

                //Course correction
                TurnLeft(1);

                if(side == KowallskiSide3796.Depot) {
                    //Run over mineral
                    DriveForward(23);
                }else
                {
                    //Run over mineral
                    DriveForward(13);
                }

                break;
        }
        //Robot should be centered at this point
        switch (side) {
            case Depot:
                telemetry.addLine("Running Depot side scripts... Cube on: " + m);
                telemetry.update();
                switch(m)
                {
                    case Right:
                        //Move away from minerals
                        DriveRight(6);

                        //Align with the depot
                        TurnLeft(35);

                        //Run to depot
                        DriveForward(11);

                        //Place marker in depot
                        PlaceMarker();

                        //Align with wall (parallel)
                        TurnRight(68);

                        //Move towards wall to ensure no minerals hit
                        DriveLeft(24);

                        //Course correction
                        TurnLeft(3);

                        //Run to crater
                        DriveBackward(50);

                        //Break crater plane
                        arm.moveUpOrDown(-0.75);
                        sleep(600);
                        arm.stopMovement();
                        break;
                    case Left:
                        //Move away from minerals
                        DriveLeft(6);

                        //Align with depot
                        TurnRight(30);

                        //Run to depot
                        DriveForward(11);

                        //Place marker
                        PlaceMarker();

                        //Align to wall
                        TurnLeft(2);

                        //Run towards wall
                        DriveLeft(12);

                        //Run to crater
                        DriveBackward(50);

                        //Break crater plane
                        arm.moveUpOrDown(-0.75);
                        sleep(600);
                        arm.stopMovement();
                        break;

                    default:
                        //Run to depot
                        DriveForward(6);

                        //Place marker in depot
                        PlaceMarker();

                        //Align with wall
                        TurnRight(35);

                        //Run to wall
                        DriveLeft(28);

                        //Course correction
                        TurnLeft(11);

                        //Run to crater
                        DriveBackward(45);

                        //Break crater plane
                        arm.moveUpOrDown(-0.75);
                        sleep(600);
                        arm.stopMovement();
                        break;
                }
                sleep(1000);
                break;
            case Crater:
                telemetry.addLine("Running Crater side scripts... Cube on: " + m);
                telemetry.update();
                //Move back to center position
                DriveBackward(13);

                //Move away from minerals (Should ensure that the robot is in the same position at
                //the end of every case if I wrote this right)
                switch(m)
                {
                    case Right:
                        DriveLeft(((2 * (ROBOT_WIDTH)) -  (2 * (L_R_MOD))) + 17);
                        break;
                    case Left:
                        DriveLeft(17);
                        break;
                    default:
                        DriveLeft((ROBOT_WIDTH - L_R_MOD) + 17);
                        break;
                }
                //Align with wall
                TurnRight(65);

                //Run to depot
                DriveBackward(50);

                //Place marker in depot
                PlaceMarker();

                //Turn back around
                TurnRight(180);

                //Drive to crater
                DriveForward(60);

                //Break crater plane
                arm.moveUpOrDown(-0.75);
                sleep(100);
                arm.stopMovement();
                break;
        }
    }

    void PlaceMarker() throws InterruptedException
    {
        telemetry.addLine("Placing the marker...");
        telemetry.update();

        //Pop ramp up
        ramp.liftOrLower(1);
        sleep(500);
        ramp.stopMovementRmp();

        //Eject Burrito
        servo.collectOrEject(-1.0);
        sleep(800);
        servo.stopMovement();

        sleep(1000);

        //Move ramp back
        ramp.liftOrLower(-1);
        sleep(500);
        ramp.stopMovementRmp();
    }

    void setup() throws InterruptedException {
        //Initialize the mechanisms
        drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
        ramp = new KowallskiRamp3796(hardwareMap.dcMotor.get("rampMotor"), hardwareMap.dcMotor.get("rampExtender"));
        arm = new KowallskiMineralArm3796(hardwareMap.dcMotor.get("collectionArmRight"), hardwareMap.dcMotor.get("collectionArmLeft"));
        servo = new KowallskiCollectionServos3796(hardwareMap.crservo.get("collectionServoRight"), hardwareMap.crservo.get("collectionServoLeft"));

        //Set up the Accelerometer/IMU
        //Initialize parameters
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        //Set angle unit to degrees
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //Set acceleration unit to m/s^2
        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Set the calibration file to the one for our IMU
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        //Enable logging
        params.loggingEnabled      = true;
        //Set the logging tag to "IMU"
        params.loggingTag          = "IMU";
        //Set the algorithm for acceleration integration (Calculus stuff to find accurate accelerations)
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //Create instance of IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Finalize initialization
        imu.initialize(params);

        //Set up the Webcam to name: "Webcam 1"
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Get view ID by the webcam name
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Initialize parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Set the license key
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //Set the camera to cameraname
        parameters.cameraName = webcamName;
        //Create instance
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Create instance of TensorFlow

        //Ensure that this will not throw an error
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            //Get the view ID
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //Initialize parameters
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            //Create instance
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            //Load models
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
    }

    public MaterialOrder checkorder() throws InterruptedException {
        // Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            updatedRecognitions = tfod.getUpdatedRecognitions();
            //Ensure we aren't acting on null (That would be an oopsies)
            if (updatedRecognitions != null) {
                //Update telemetry with the number of objects detected
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                //If the size is 3, it can just return a straight up order and act as normal
                if (updatedRecognitions.size() == 3) {
                    //Ensure all of the positions have a "zero"
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    //Loop through to find out the positions of all of the minerals
                    for (Recognition recognition : updatedRecognitions) {
                        //If it's gold, set the gold position to its position
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                        //If it's silver, set the silver position to its position
                        else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    //Ensure that all three are seen
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        //Gold on Left, Gold mineral's position less than silver 1 and 2
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            return MaterialOrder.Left;
                        }
                        //Gold on Right, Gold mineral's position more than silver 1 and 2
                        else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            return MaterialOrder.Right;
                        }
                        //Gold in center or err, Gold mineral's position between 1 and 2 in best case
                        else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            return MaterialOrder.Center;
                        }
                    }
                }
                //Update telemetry
                telemetry.update();
            }
        }
        //If all else fails, return unknown
        return MaterialOrder.Unknown;
    }

    //Simple conversion calculation to standardize our encoders
    public int inchesToTicks(double inches, MotorSide side) throws InterruptedException {
        //Either side can be different based on which motor you want to get the ticks from
        switch (side) {
            case LeftMotor:
                return (int) ((inches * TICKS_PER_INCH_L));
            case RightMotor:
                return (int) ((inches * TICKS_PER_INCH_R));
            default:
                return -1;
        }

    }

    //Simple conversion calculation to standardize our encoders
    public double ticksToInches(int ticks, MotorSide side) throws InterruptedException {
        //Convert to doubles to prevent rounding issues
        double t = (double) ticks;

        //Either side can be different based on which motor you want to get the ticks from
        switch (side) {
            case LeftMotor:
                return (int) ((t / TICKS_PER_INCH_L));
            case RightMotor:
                return (int) ((t / TICKS_PER_INCH_R));
            default:
                return -1;
        }
    }

    //All four of these methods do the same thing with slight variations (The order of the motors based
    //on the direction it's moving). All of the comments and descriptions are in the forward method
    public void DriveForward(double inches) throws InterruptedException {
        //Set the order of the motors
        drive.rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotor.Direction.FORWARD);

        //Set the encoders to be used for this drive method
        drive.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Update telemetry...
        telemetry.addLine("Forward...");
        telemetry.update();

        //Set the target positions
        drive.leftBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.LeftMotor));
        drive.rightBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.RightMotor));

        //Set all of the wheels to the AUTO_POWER constant
        drive.leftBackWheel.setPower(AUTO_POWER);
        drive.rightBackWheel.setPower(AUTO_POWER);
        drive.leftFrontWheel.setPower(AUTO_POWER);
        drive.rightFrontWheel.setPower(AUTO_POWER);

        //Wait until the drive is done driving
        while (opModeIsActive() && (drive.rightBackWheel.isBusy() && drive.leftBackWheel.isBusy())) {
            //Accelerate the drive too, just to make it a bit faster
            if(drive.leftFrontWheel.getPower() < ACCELERATION_MAX) {
                drive.leftFrontWheel.setPower(drive.leftFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.rightFrontWheel.setPower(drive.rightFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.leftBackWheel.setPower(drive.leftBackWheel.getPower() + ACCELERATION_VALUE);
                drive.rightBackWheel.setPower(drive.rightBackWheel.getPower() + ACCELERATION_VALUE);
            }
            telemetry.addLine("Left Status: " + ticksToInches(drive.leftBackWheel.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
            telemetry.addLine("Right Status: " + ticksToInches(drive.rightBackWheel.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
            telemetry.update();
        }

        //Stop the robot after its done driving
        drive.leftBackWheel.setPower(0);
        drive.leftFrontWheel.setPower(0);
        drive.rightBackWheel.setPower(0);
        drive.rightFrontWheel.setPower(0);

        //Set the motors back to how they were
        drive.leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriveBackward(double inches) throws InterruptedException {
        drive.rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.leftBackWheel.setDirection(DcMotor.Direction.REVERSE);

        drive.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Backward...");
        telemetry.update();

        drive.leftBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.LeftMotor));
        drive.rightBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.RightMotor));

        drive.leftBackWheel.setPower(AUTO_POWER);
        drive.rightBackWheel.setPower(AUTO_POWER);
        drive.leftFrontWheel.setPower(AUTO_POWER);
        drive.rightFrontWheel.setPower(AUTO_POWER);

        while (opModeIsActive() && (drive.rightBackWheel.isBusy() && drive.leftBackWheel.isBusy())) {
            if(drive.leftFrontWheel.getPower() < ACCELERATION_MAX) {
                drive.leftFrontWheel.setPower(drive.leftFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.rightFrontWheel.setPower(drive.rightFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.leftBackWheel.setPower(drive.leftBackWheel.getPower() + ACCELERATION_VALUE);
                drive.rightBackWheel.setPower(drive.rightBackWheel.getPower() + ACCELERATION_VALUE);
            }
            telemetry.addLine("Left Status: " + ticksToInches(drive.leftBackWheel.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
            telemetry.addLine("Right Status: " + ticksToInches(drive.rightBackWheel.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
            telemetry.update();
        }

        drive.leftBackWheel.setPower(0);
        drive.leftFrontWheel.setPower(0);
        drive.rightBackWheel.setPower(0);
        drive.rightFrontWheel.setPower(0);

        drive.leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriveLeft(double inches) throws InterruptedException {
        drive.rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.leftBackWheel.setDirection(DcMotor.Direction.FORWARD);

        drive.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Left...");
        telemetry.update();

        drive.leftBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.LeftMotor));
        drive.rightBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.RightMotor));

        drive.leftBackWheel.setPower(AUTO_POWER);
        drive.rightBackWheel.setPower(AUTO_POWER);
        drive.leftFrontWheel.setPower(AUTO_POWER);
        drive.rightFrontWheel.setPower(AUTO_POWER);

        while (opModeIsActive() && (drive.rightBackWheel.isBusy() && drive.leftBackWheel.isBusy())) {
            if(drive.leftFrontWheel.getPower() < ACCELERATION_MAX) {
                drive.leftFrontWheel.setPower(drive.leftFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.rightFrontWheel.setPower(drive.rightFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.leftBackWheel.setPower(drive.leftBackWheel.getPower() + ACCELERATION_VALUE);
                drive.rightBackWheel.setPower(drive.rightBackWheel.getPower() + ACCELERATION_VALUE);
            }
            telemetry.addLine("Left Status: " + ticksToInches(drive.leftBackWheel.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
            telemetry.addLine("Right Status: " + ticksToInches(drive.rightBackWheel.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
            telemetry.update();
        }

        drive.leftBackWheel.setPower(0);
        drive.leftFrontWheel.setPower(0);
        drive.rightBackWheel.setPower(0);
        drive.rightFrontWheel.setPower(0);

        drive.leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriveRight(double inches) throws InterruptedException {
        drive.rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotor.Direction.REVERSE);


        drive.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Right...");
        telemetry.update();

        drive.leftBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.LeftMotor));
        drive.rightBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.RightMotor));

        drive.leftBackWheel.setPower(AUTO_POWER);
        drive.rightBackWheel.setPower(AUTO_POWER);
        drive.leftFrontWheel.setPower(AUTO_POWER);
        drive.rightFrontWheel.setPower(AUTO_POWER);

        while (opModeIsActive() && (drive.rightBackWheel.isBusy() && drive.leftBackWheel.isBusy())) {
            if(drive.leftFrontWheel.getPower() < ACCELERATION_MAX) {
                drive.leftFrontWheel.setPower(drive.leftFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.rightFrontWheel.setPower(drive.rightFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.leftBackWheel.setPower(drive.leftBackWheel.getPower() + ACCELERATION_VALUE);
                drive.rightBackWheel.setPower(drive.rightBackWheel.getPower() + ACCELERATION_VALUE);
            }
            telemetry.addLine("Left Status: " + ticksToInches(drive.leftBackWheel.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
            telemetry.addLine("Right Status: " + ticksToInches(drive.rightBackWheel.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
            telemetry.update();
        }

        drive.leftBackWheel.setPower(0);
        drive.leftFrontWheel.setPower(0);
        drive.rightBackWheel.setPower(0);
        drive.rightFrontWheel.setPower(0);

        drive.leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void TurnLeft(double degrees) throws InterruptedException{
        //Speed should be double auto power as it's turning
        double speed = AUTO_POWER * 2.0;
        //Reset angle to zero
        resetAngle();
        //Get angles and set it to angles
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Find target angle
        double target = degrees + angles.firstAngle;

        //Ensure fastest route
        if(target > 180){
            target -= 360;
        }else if(target < -180){
            target += 360;
        }

        //Ensure fastest route
        if(target < angles.firstAngle) {
            //Turn until it's at the angle
            while (opModeIsActive() && (target < angles.firstAngle)) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                drive.leftFrontWheel.setPower(speed);
                drive.leftBackWheel.setPower(speed);
                drive.rightBackWheel.setPower(speed);
                drive.rightFrontWheel.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        //Ensure fastest route
        else if(target > angles.firstAngle) {
            //Turn until it's at the angle
            while (opModeIsActive() && (target > angles.firstAngle)) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                drive.leftFrontWheel.setPower(-speed);
                drive.leftBackWheel.setPower(-speed);
                drive.rightBackWheel.setPower(-speed);
                drive.rightFrontWheel.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        drive.stopMovement();
        sleep(50);
    }

    void TurnRight(double degrees) throws InterruptedException {
        double speed = AUTO_POWER * 2.0;
        resetAngle();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle - degrees;
        if(target > 180){
            target -= 360;
        }else if(target < -180){
            target += 360;
        }
        if(target < angles.firstAngle) {
            while (opModeIsActive() && (target < angles.firstAngle)) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                drive.leftFrontWheel.setPower(speed);
                drive.leftBackWheel.setPower(speed);
                drive.rightBackWheel.setPower(speed);
                drive.rightFrontWheel.setPower(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }else if(target > angles.firstAngle) {
            while (opModeIsActive() && (target > angles.firstAngle)) {
                telemetry.addData("Target:", target);
                telemetry.addData("Angle:", angles.firstAngle);
                telemetry.update();
                drive.leftFrontWheel.setPower(-speed);
                drive.leftBackWheel.setPower(-speed);
                drive.rightBackWheel.setPower(-speed);
                drive.rightFrontWheel.setPower(-speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        drive.stopMovement();
        sleep(50);
    }

    //Reset angles to zero
    void resetAngle() throws InterruptedException{
        //Get the imu's position
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Set global angle to 0
        globalAngle = 0;
    }
}