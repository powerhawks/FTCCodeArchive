package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class JAWLAutonomous3796 extends LinearOpMode{
    Side3796 side;

    /*
    *30 Seconds Limit
    *
    *--------------------------------------------
    *The order in which things should be done
    *
    * Grab glyph
    * Lift Glyph
    * Drop Jewel Arm --
    * Look at color -- can throw error
    * Scan Vumark -- can throw error
    * Rotate/move to knock jewel
    * Lift arm
    * Reverse rotate
    *
    * and finally...
    *
    * ++++Navigate to cryptobox++++
    *
    * -------------------------------------------
    *
    * A suggestion is to make this autonomous mode stateful, that is, the program runs in states
    * Apparently there are problem with running the opModeIsActive while loop like a function so we must run in a stateful sense
    * Our states will be the following: GetGlyph, LiftGlyph, ScanColor, ScanVumark, KnockJewel, FinishOnPlatform, Navigate
    * We removed FinishOnPlatform
    *
    */

    @Override
    public void runOpMode() throws InterruptedException{

        //Create and initialize all component objects
        JAWLDrive3796 drive = new JAWLDrive3796(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        drive.setEncoders(true);
        JAWLGrabber3796 grabber = new JAWLGrabber3796(hardwareMap.servo.get("left_arm"), hardwareMap.servo.get("right_arm"));
        JAWLLift3796 lift = new JAWLLift3796(hardwareMap.dcMotor.get("lift_motor"));
        JAWLColorArm3796 colorArm = new JAWLColorArm3796(hardwareMap.servo.get("color_arm"));
        JAWLColorSensor3796 colorDistanceSensor = new JAWLColorSensor3796(hardwareMap.get(ColorSensor.class, "color_distance"), hardwareMap.get(DistanceSensor.class, "color_distance"));

        //Statements for initializing vuforia
        //We create these here to minimize delay during the actual opmode
        VuforiaLocalizer vuforia;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AX+0L7z/////AAAAGfsI0P59QEr1irbabDfmd5CDbjk/PlQiQawZBzkdK2Jcf97SwbDegG8S9JaJpxv7iR9Ziq21efhfRW/WHkAciKM6qLR2jdQtZypgHWWo0ZnkyrDDQ1CxZPz1pAmPGOJ8DzTEb/x/700NwOVLtvkiCTrBD9Ld7vq2Kl150/apUzw4kaIYBIAd8fJ42S+30JYrs2UasrwaGeViNlGpWE+DxRERvrNLLu4pEUtWQf2Z4BagDO4H7WXiFtFe6pU7/m3PUCUCiKTSu0NtKTHdj0MebUeCfohHUrxWEBPXNPRYI3CS8YypOti7+hYusv51lUpNESImH5guK07ErN+3hV7LBG1qVbjueLfNW++kzS7IVr+u\n" +
                "\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //Load vuforia trackable
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        //Variables for loop
        boolean autoFinish = false;
        AutoState state = AutoState.GetGlyph;
        String color = "UR BAD";
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        //Here we reset the encoders on the motors
        //This is very important to do as old encoder values could mess up navigation
        JAWLRotationNavigation3796.resetNavigation(drive);
        sleep(300);
        drive.setEncoders(true);

        /*
        For now, while we are testing, I will wait one second before every state change for debugging purposes
         */
        waitForStart();
        relicTrackables.activate();

        try {
            while (opModeIsActive() && !autoFinish) {
                //This handles our states

                switch (state) {
                    case GetGlyph:
                        //Here we grab the glyph that was placed in front of the robot during setup
                        //We also lower the color arm to scan the color in the next state
                        grabber.rightArmClose();
                        grabber.leftArmClose();
                        //sleep(2000);
                        state = AutoState.ScanColor;
                        telemetry.addData("State", state);
                        break;
                    case ScanColor:
                        //Now we swing the color arm down and scan the color of the ball
                        colorArm.armDown();
                        sleep(1000);
                        color = colorDistanceSensor.getColor();
                        sleep(500);
                        if (color.equals("UR BAD")) {
                            //Tell the driver station that no color was detected
                            telemetry.addLine("No color found!!");
                            colorArm.armUp();
                        } else {
                            telemetry.addData("Color", color);
                        }
                        state = AutoState.ScanVumark;
                        telemetry.addData("State", state);
                        break;
                    case ScanVumark:
                        //Here we scan the vumark placed next to the jewels
                        sleep(500);
                        vuMark = RelicRecoveryVuMark.from(relicTemplate);
                        sleep(200);
                        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                            telemetry.addLine("No vumark found!!");
                        } else {
                            telemetry.addData("VuMark", vuMark);
                        }
                        relicTrackables.deactivate();
                        state = AutoState.KnockJewel;
                        telemetry.addData("State", state);
                        break;
                    case KnockJewel:
                        //In this autonomous state we knock the opposing team's jewel off the platform
                        //How many ticks we turn for
                        int turnTicks = 200;
                        //The power for the turns
                        double turnPower = 0.5;

                        if (side == Side3796.Red1 || side == Side3796.Red2) {
                            if (color.equals("BLUE")) {
                                //If we are on platform Red1 and we scanned the ball to be red, we want to knock the front ball b
                                drive.turnRight(turnTicks, turnPower);
                                sleep(1500);
                                colorArm.armUp();
                                sleep(500);
                                drive.turnLeft(turnTicks, turnPower);
                                sleep(1500);
                            } else if (color.equals("RED")) {
                                //If we are on platform Red1 and we scanned the ball to be blue, we want to knock the back ball
                                drive.turnLeft(turnTicks, turnPower);
                                sleep(1500);
                                colorArm.armUp();
                                sleep(500);
                                drive.turnRight(turnTicks, turnPower);
                                sleep(1500);
                            }
                        } else {
                            if (color.equals("RED")) {
                                //If we are on platform Red1 and we scanned the ball to be red, we want to knock the front ball b
                                drive.turnRight(turnTicks, turnPower);
                                sleep(1500);
                                colorArm.armUp();
                                sleep(500);
                                drive.turnLeft(turnTicks, turnPower);
                                sleep(1500);
                            } else if (color.equals("BLUE")) {
                                //If we are on platform Red1 and we scanned the ball to be blue, we want to knock the back ball
                                drive.turnLeft(turnTicks, turnPower);
                                sleep(1500);
                                colorArm.armUp();
                                sleep(500);
                                drive.turnRight(turnTicks, turnPower);
                                sleep(1500);
                            }
                        }

                        /*
                        Programming note:
                        Waiting for a specific time until the motors are done moving backwards or forwards is probably a bad idea
                        Motor time depends on battery power an many other factors
                        If we have time we should develop a better solution
                        -Joe
                        ToDo:maybe develop a better solution for this if we have time
                        */
                        sleep(1000);
                        state = AutoState.LiftGlyph;
                        telemetry.addData("State", state);
                        break;
                    case LiftGlyph:
                        //Here we lift the glyph we grabbed off the ground slightly to prevent it from dragging
                        //We only run the lift motor for 50 ms
                        //Precision is not important here
                        lift.moveMotor(0.5);
                        sleep(500);
                        lift.moveMotor(0.1466);
                        state = AutoState.Navigate;
                        telemetry.addData("State", state);
                        break;
                    case Navigate:
                        //Here we navigate to the cryptobox!
                        //We have written another method for this to clean up the code
                        if(!JAWLRotationNavigation3796.isDone) {
                            JAWLRotationNavigation3796.navigateToCryptobox(side, vuMark, drive, telemetry);
                        } else {
                            sleep(1000);
                            state = AutoState.DropGlyph;
                            telemetry.addData("State", state);
                        }
                        break;
                    case DropGlyph:
                        lift.moveMotor(0);
                        grabber.leftArmShort();
                        grabber.rightArmShort();
                        if(vuMark == RelicRecoveryVuMark.RIGHT) {
                            grabber.rightArmOpen();
                        }
                        state = AutoState.Backup;
                        telemetry.addData("State", state);
                        break;
                    case Backup:
                        drive.backwards(200, 0.5);
                        sleep(1000);

                        //We're finished the autonomous!


                        grabber.leftArmOpen();
                        grabber.rightArmOpen();

                        autoFinish = true;
                        break;
                }

                telemetry.update();
            }
        } finally {
            stopRobot();
        }
    }

    private enum AutoState {
        GetGlyph, LiftGlyph, ScanColor, ScanVumark, KnockJewel, Navigate, DropGlyph, Backup
    }

    private void stopRobot() {
        //This solution is bad
        hardwareMap.dcMotor.get("left_drive").setPower(0);
        hardwareMap.dcMotor.get("right_drive").setPower(0);
        hardwareMap.dcMotor.get("lift_motor").setPower(0);
    }
}
