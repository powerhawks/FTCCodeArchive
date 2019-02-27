package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Pierce on 12/5/17.
 */

public abstract class MBaseAutoMode3583 extends LinearOpMode {

    // constants
    static final int MAX_VUFORIA_ATTEMPTS = 5;
    static final int MAX_VUFORIA_WAIT_BETWEEN_ATTEMPTS = 500; // ms
    static final double     COUNTS_PER_MOTOR_REV    = 307.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ; // This is < 1.0 if geared UP
                                                            // 24 -> 16 teeth
    static final double     WHEEL_DIAMETER_INCHES   = 3.9375 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;


    // DcMotors
    static String LEFT_REAR_MOTOR = "lBackDrive";
    static String LEFT_FRONT_MOTOR = "lFrontDrive";
    static String RIGHT_REAR_MOTOR     = "rRackDrive";
    static String RIGHT_FRONT_MOTOR = "rFrontDrive";
    static String GLYPH_LIFT_MOTOR = "gylphLift";
    static String RELIC_ARM_MOTOR           = "relicArm";
    static String RELIC_ARM_ANGLE_MOTOR     = "relicArmAngle";

    // Servos
    static String GLYPH_CLAMP_RIGHT_SERVO = "rGlyphClamp";
    static String GLYPH_CLAMP_LEFT_SERVO  = "lGlyphClamp";
    static String RELIC_CLAMP_RIGHT_SERVO = "rRelicClamp";
    static String RELIC_CLAMP_LEFT_SERVO  = "lRelicClamp";
    static String JEWEL_ARM_SERVO = "jewelServo";

    // Sensors
    static String JEWEL_COLOR_SENSOR = "jewelColorSensor";

    DcMotor leftRearMotor, leftFrontMotor, rightRearMotor, rightFrontMotor, glyphLiftMotor,
            relicArmMotor, relicArmAngleMotor;

    Servo jewelArmServo, rightGlyphClampServo, leftGlyphClampServo, leftRelicClampServo,
            rightRelicClampServo;

    ColorSensor jewelArmColorSensor;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    enum JewelColor {
        RED,
        BLUE,
        UNKNOWN
    }

    public void initRobot() {
        // get motors from hardwareMap
        leftRearMotor = hardwareMap.dcMotor.get(LEFT_REAR_MOTOR);
        leftFrontMotor = hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR);
        rightRearMotor = hardwareMap.dcMotor.get(RIGHT_REAR_MOTOR);
        rightFrontMotor = hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR);
        glyphLiftMotor = hardwareMap.dcMotor.get(GLYPH_LIFT_MOTOR);
        relicArmMotor = hardwareMap.dcMotor.get(RELIC_ARM_MOTOR);
        relicArmAngleMotor = hardwareMap.dcMotor.get(RELIC_ARM_ANGLE_MOTOR);

        // get servos from hardwareMap
        jewelArmServo = hardwareMap.servo.get(JEWEL_ARM_SERVO);
        rightGlyphClampServo = hardwareMap.servo.get(GLYPH_CLAMP_RIGHT_SERVO);
        leftGlyphClampServo = hardwareMap.servo.get(GLYPH_CLAMP_LEFT_SERVO);
        leftRelicClampServo = hardwareMap.servo.get(RELIC_CLAMP_LEFT_SERVO);
        rightRelicClampServo = hardwareMap.servo.get(RELIC_CLAMP_RIGHT_SERVO);

        // get sensors
        jewelArmColorSensor = hardwareMap.colorSensor.get(JEWEL_COLOR_SENSOR);

        // init vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    // u2se Vuforia to examine vuMark to determine which column the glyph needs to go into.
    RelicRecoveryVuMark determineVumarkPosition() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        return vuMark;
    }

    JewelColor determineJewlelColor() {

        JewelColor jewelColor = JewelColor.UNKNOWN;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // convert the RGB values to HSV values.
        Color.RGBToHSV(
                (jewelArmColorSensor.red() * 255) / 800,
                (jewelArmColorSensor.green() * 255) / 800,
                (jewelArmColorSensor.blue() * 255) / 800, hsvValues);


        // TODO find correct HUE values for RED and BLUE

        if (hsvValues[0] < 10 && hsvValues[0] > 250) {
            jewelColor = JewelColor.RED;
        } else if (hsvValues[0] > 120 && hsvValues[0] < 180) {
            jewelColor = JewelColor.BLUE;
        }

        return jewelColor;
    }

    public void goForwardInch(double inch) throws InterruptedException {
        encoderDrive(DRIVE_SPEED, inch, inch, 0.5);
    }

    public void goLeftInch(double inch) throws InterruptedException {
        encoderDrive(TURN_SPEED, inch * -1, inch, 0.5);
    }

    public void goRightInch(double inch) throws InterruptedException {
        encoderDrive(TURN_SPEED, inch, inch * -1, 0.5);
    }

    public void goBackwardsInch(double inch)throws InterruptedException {
        encoderDrive(DRIVE_SPEED, inch * -1, inch * -1, 0.5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftRearMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightRearMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftRearMotor.setTargetPosition(newLeftTarget);
            rightRearMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setPower(speed);
            rightRearMotor.setPower(speed);

            while (leftRearMotor.isBusy() || rightRearMotor.isBusy() ) {
                // do something
            }

            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    void grabGlyph() {
        // TODO verify these numbers
        leftGlyphClampServo.setPosition(.3);
        rightGlyphClampServo.setPosition(0);
    }

    void releaseGlyph() {
        // TODO verify these numbers
        leftGlyphClampServo.setPosition(0);
        rightGlyphClampServo.setPosition(.4);
    }

}
