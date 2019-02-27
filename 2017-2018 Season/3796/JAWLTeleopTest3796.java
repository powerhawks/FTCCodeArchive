package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * November 18, 2017
 * Created by Chase Galey and Joe Lewis
 * Tony the Train that Tried Test Teleop Class
 *
 * For testing components and new code
 * NOT FOR COMPETITION USE
 */


@TeleOp(name = "Old Teleop Tester")
public class JAWLTeleopTest3796 extends LinearOpMode {

    //ToDo: Add telemetry reports to opmode
    //If we were in competition, this is what defining our side would look like:
    //Side3796 side = Side3796.Red2;

    public void runOpMode() {

        //Create all component objects
        //Create Drive Object (note: left then right for implicit arguments)
        JAWLDrive3796 drive = new JAWLDrive3796(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        //Set the drive motors to run without encoders
        drive.setEncoders(false);
        //Create Grabber object
        JAWLGrabber3796 grabber = new JAWLGrabber3796(hardwareMap.servo.get("left_arm"), hardwareMap.servo.get("right_arm"));
        //Create Lift object
        JAWLLift3796 lift = new JAWLLift3796(hardwareMap.dcMotor.get("lift_motor"));
        //Create color arm object
        JAWLColorArm3796 colorArm = new JAWLColorArm3796(hardwareMap.servo.get("color_arm"));
        //Creates color sensor name
        ColorSensor colorSensorTemp = hardwareMap.get(ColorSensor.class, "color_distance");
        //Creates distance sensor name
        DistanceSensor distanceSensorTemp = hardwareMap.get(DistanceSensor.class, "color_distance");
        //Creates the color-distance sensor object
        JAWLColorSensor3796 colorDistanceSensor = new JAWLColorSensor3796(colorSensorTemp, distanceSensorTemp);
        //Creates the variable that will hold the color of the jewel
        String colorOfBall;
        //Creates relic arm objects
        //TTTTRelicArm3796 relicArm = new TTTTRelicArm3796(hardwareMap.dcMotor.get("relic_up_down"), hardwareMap.dcMotor.get("relic_out_in"), hardwareMap.dcMotor.get("relic_grab"));

        //Set color arm up

        waitForStart();

        colorArm.armUp();

        while (opModeIsActive()) {
            //Here is where we define how the controllers should work
            //In theory, no logic retaining to controlling the components should be in here

            /**
             * Drive
             */

            //drive.leftDrive(-gamepad1.left_stick_y);
            //drive.rightDrive(-gamepad1.right_stick_y);

            drive.leftDrive(-gamepad1.left_stick_y);
            drive.rightDrive(-gamepad1.right_stick_y);

            /**
             * Lift
             */
            telemetry.addData("Gamepad2 Right Stick Y", -gamepad2.right_stick_y);
            if (-gamepad2.right_stick_y > 0) {
                lift.moveMotor(-gamepad2.right_stick_y);
                telemetry.addData("Lift Power", 1);
            }else if(-gamepad2.right_stick_y < 0){
                lift.moveMotor(-0.1);
                telemetry.addData("Lift Power", -0.1);
            }else{
                lift.moveMotor(0);
                telemetry.addData("Lift Power", 0);
                telemetry.addData("Set to zero power", "YES");
            }

            /**
             * Color Arm
             */
            if (gamepad2.a) colorArm.armDown();
            if (gamepad2.b) colorArm.armUp();

            /**
             * Color Sensor
             */
            if(gamepad2.x){
                colorOfBall = colorDistanceSensor.getColor();
//                if(colorOfBall.equals("RED")){
//                    telemetry.addData("Red", 1);
//                }else if (colorOfBall.equals("BLUE")){
//                    telemetry.addData("Blue", 1);
//                }else{
//                    telemetry.addData("Error", 1);
//                }

                telemetry.addData(colorOfBall, 0);

                telemetry.addData("Distance", "" + colorDistanceSensor.getDistance());
            }


            /**
             * Grabbers
             */
            if (gamepad2.left_trigger > 0) {
                //Closes the left arm, then displays the position in telemetry
                grabber.leftArmClose();
                double a = grabber.leftPosition();
                telemetry.addData("Left", a);
            }
            if (gamepad2.left_bumper) {
                //Opens the left arm, then displays the position in telemetry
                grabber.leftArmOpen();
                double b = grabber.leftPosition();
                telemetry.addData("Left", b);
            }

            if (gamepad2.right_trigger > 0) {
                //Opens the right arm, then displays the position in telemetry
                grabber.rightArmClose();
                double c = grabber.rightPosition();
                telemetry.addData("Right", c);
            }

            if (gamepad2.right_bumper) {
                //Closes the right arm, then displays the position in telemetry
                grabber.rightArmOpen();
                double d = grabber.rightPosition();
                telemetry.addData("Right", d);
            }
            //Variable control over grabbers
/*

            if (gamepad2.right_stick_x != 0) {
                grabber.rightArmMove(gamepad2.right_stick_x);
                double i = grabber.rightPosition();
                telemetry.addData("Right", i);
                telemetry.update();
            }

            if (gamepad2.left_stick_x != 0) {
                grabber.leftArmMove(gamepad2.left_stick_x);
                double j = grabber.leftPosition();
                telemetry.addData("Left", j);
                telemetry.update();
            }

*/

            /**
             * Relic Arm
             */

            /*
            if(-gamepad2.left_stick_y > 0) {
                relicArm.tiltUpOrDown(-gamepad2.left_stick_y);
            }else if(-gamepad2.left_stick_y < 0) {
                relicArm.tiltUpOrDown(-gamepad2.left_stick_y);
            }
            if(gamepad2.dpad_up) {
                relicArm.moveLeftRight(1);
            }else if(gamepad2.dpad_down) {
                relicArm.moveLeftRight(-1);
            }
            if(gamepad2.dpad_left) {
                relicArm.grabRelic(1);
            }
            */

            //Update all of our telemetries
            telemetry.update();
        }
    }
}