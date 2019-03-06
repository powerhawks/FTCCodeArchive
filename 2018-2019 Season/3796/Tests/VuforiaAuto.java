package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Enums.KowallskiSide3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;
import org.firstinspires.ftc.teamcode.HelperClasses.Transform;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;

/**
 * November 3, 2018
 * Lincoln Doney and Chase Galey
 * Autonomous Vuforia code
 ---------------------------------
 * PSEUDOCODE:
 *      Wait for start and set up vuforia globals and constants
 *
 *  NOTE: Infinitley runs
 *      Find objects + Set their transforms to an array
 *      If targets are found
 *          Find the transform of the robot based off the targets' location
 *          Display location of robot to telemetry
 *          With location, run autonomous responses
 --------------------------------
 * Transform class documentation
 *  Description:
 *      The transform class simplifies the storing of the transform of the robot
 *
 *  Constructor:
 *      - Parameters
 *          * VectorF transform of object
 *          * Orientation rotation of object
 *
 *  Variables
 *      - PUBLIC:
 *         * posX X Position of object
 *         * posY Y Position of object
 *         * posZ Z Position of object
 *         * rotX X Rotation of object
 *         * rotY Y Rotation of object
 *         * rotZ Z Rotation of object
 */
@Disabled
public class VuforiaAuto extends LinearOpMode {
    /**
        ************************
        Constants declared here:
        ************************
     */
    //Vuforia License Key
    private static final String VUFORIA_KEY = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n" +
            "\n";
    //Constants for unit conversions
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
    //***END OF CONSTANTS DECLARATION***

    /**
        *******************************
        Global Variables declared here:
        *******************************
     */

    //Defining all of the global variables
    //NOTE: All of them are initialized in setup();

    VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation;
    List<VuforiaTrackable> allTrackables;
    VuforiaTrackables targetsRoverRuckus;
    VuforiaLocalizer.Parameters parameters;
    boolean targetVisible;
    OpenGLMatrix phoneLocation = null;

    KowallskiSide3796 orientation;

    //Declares all of the Vuforia Trackable objects (To be initialized in setup();
    VuforiaTrackable backSpace = null;
    VuforiaTrackable frontCraters = null;
    VuforiaTrackable redFootprint = null;
    VuforiaTrackable blueRover = null;
    VuforiaTrackable visibleVuMark = null;

    //Camera displacement constants for robot
    //      NOTE: GET BUILD TO GIVE US VALUES (IN MM)
    final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    KowallskiMecanumDrive3796 drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));

    //Mecanum Drive Declaration

    //MecanumDrive3796 drive = new MecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
    /***END OF GLOBAL FIELD DECLARATION***/

    /**
        *************************
        ENTRY POINT (runOpMode())
        *************************
     */
    @Override
    public void runOpMode() throws InterruptedException {
        /**
                ******************************
                Runs ONCE after program starts
                ******************************
         */
        waitForStart();
        setUp();
        /***End of one-time code***/

        while(opModeIsActive())
        {
            /**
                    ************************************
                    Runs infinitley after program starts
                    ************************************
             */

            //Finds objects and sets transforms
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    visibleVuMark = trackable;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            //If the target is found, it processes the data
            if (targetVisible) {
                //Finds translation of the robot
                VectorF translation = lastLocation.getTranslation();
                //Finds orientation of the robot
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //Stores transform and rotation of the object into Transform object to be later parsed
                Transform locPic = new Transform(translation, rotation){{
                    posX = trans.get(0) / mmPerInch;
                    posY = trans.get(1) / mmPerInch;
                    posZ = trans.get(2) / mmPerInch;
                    rotX = rot.firstAngle;
                    rotY = rot.secondAngle;
                    rotZ = rot.thirdAngle;
                }};
                //Updates telemetry with the locational data
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        locPic.posX, locPic.posY, locPic.posZ);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f,  %.0f, %.0f",
                        locPic.rotX, locPic.rotY, locPic.rotZ);
                if (lastLocation != null) {
                    //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    telemetry.addData("Pos", lastLocation.formatAsTransform());
                } else {
                    telemetry.addData("Pos", "Unknown");
                }
            /**                  DO STUFF HERE                    */



            /**                 DON'T DO STUFF PAST HERE           */
            }
            else {
                //If no object is found, tell telemetry
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        /***End of infinite loop***/
    }

    //SETUP METHOD FROM THE PRE-MADE FTC CODE, DO NOT TOUCH
    private void setUp()
    {
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;

        allTrackables = new ArrayList<VuforiaTrackable>();
        lastLocation = null;
        targetVisible = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        targetsRoverRuckus.activate();
        blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        phoneLocation = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                VuforiaLocalizer.CameraDirection.BACK == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }


    }
}


