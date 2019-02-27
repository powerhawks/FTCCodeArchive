package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * January 7, 2018
 * Created by Chase Galey and Joe Lewis
 *
 * New teleop class for use in competitions
 */

@TeleOp(name = "Competition Teleop")
public class JAWLCompetitionTeleop3796 extends LinearOpMode{

    public void runOpMode(){

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
        //We never ended up having a relic arm!
        //The lift helper will set the idle power of the motor to a small double, keeping the lift from idly falling down
        boolean liftHelper = false;
        int i = 0;

        waitForStart();

        colorArm.armUp();

        while (opModeIsActive()) {
            //Here is where we define how the controllers should work
            //In theory, no logic retaining to controlling the components should be in here

            /*
             * Drive
             */

            //The left drive wheel is controlled by the opposite value of the left stick's y value on the first gamepad
            //The right drive is the same way except with the right drive wheel
            drive.leftDrive(-gamepad1.left_stick_y);
            drive.rightDrive(-gamepad1.right_stick_y);

            /*
             * Lift
             */
            telemetry.addData("Gamepad2 Right Stick Y", -gamepad2.right_stick_y);

            if (-gamepad2.right_stick_y > 0) {
                //First checks to see if the right stick's negative y value is greater then zero.
                lift.moveMotor(-gamepad2.right_stick_y);
                //If it is, it sets the power for the motor to 1, and adds telemetry
                telemetry.addData("Lift Power", 1);
            } else if (-gamepad2.right_stick_y < 0) {
                //Checks if the negative value of the right right stick's y position is less than zero
                lift.moveMotor(-0.1);
                //Sets the power for the motor to -0.1, and adds telemetry
                telemetry.addData("Lift Power", -0.1);
            } else {
                //We check to see if the liftHelper is enabled
                if(liftHelper) {
                    lift.moveMotor(0.1466 );
                } else {
                    lift.moveMotor(0);
                }
            }



            /*
             * Lift helper control
             */

            if(gamepad2.a) {
                if(i == 0) {
                    liftHelper = !liftHelper;
                }
                i = 5;
            }

            if(i != 0) {
                i--;
            }

            telemetry.addData("Lift Helper Enabled", liftHelper);



            /*
             * Grabbers
             */
            if (gamepad2.left_trigger > 0) {
                //Closes the left arm, then displays the position in telemetry
                grabber.leftArmClose();
                double a = grabber.leftPosition();
                //Adds telemetry to display positions of grabbers, mostly for testing, but can be useful later on
                telemetry.addData("Left", a);
            }

            if (gamepad2.left_bumper) {
                //Opens the left arm, then displays the position in telemetry
                grabber.leftArmOpen();
                double b = grabber.leftPosition();
                //We made the variables different as to avoid any and all possible errors
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

            if (gamepad2.dpad_left){
                //Closes the left arm to a shorter distance
                grabber.leftArmShort();
            }

            if(gamepad2.dpad_right){
                //Closes the right arm to a shorter distance
                grabber.rightArmShort();

            }

            //Update all of our telemetries at once so we can see all of it at the same time
            telemetry.update();
        }
    }
}
