package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiCollectionServos3796;

/**
 * Made on October 16, 2018
 * Chase Galey and Lincoln Doney
 * Was previously a test TeleOp class
 */

/**
 * PSUEDO CODE:
 *  Since we made separate classes for each component we must create and object for each one.
 *  We must get the correct device from the configuration to be able to use them later on.
 *
 *  We make a variable for each button that will be pressed.
 *  This way we do not have to type negatives and periods every time we want to add something
 *
 *  We have four "if" statements at the top of our code to prevent anything from moving when we do not want it to.
 *
 *  We have an "if" statement for the ramp here. If the left stick on the second game pad is not in it's resting position, the ramp will move.
 *  We have two more if statements for the collection system here, they will make the servos spin inwards or outwards, depending on which trigger in pulled.
 *  Next are the if statements for the arm. The A button will make it go down at half speed, the y button at 3/4 speed, the x button at 1/4 speed, and the b button will make it go up at half speed.
 *
 *  The Dpad will move the robot diagonally in the diagonal that is pressed.
 *  Not pressing the left bumper and using the left joystick will make the robot go forwards or backwards.
 *  Pressing the left bumper and using the left joystick will make the robot go side to side.
 *  Using the right joystick will turn the robot in the desired direction.
 */

@TeleOp (name="Collection Servo Test", group = "X")
@Disabled
public class KowallskiCollectorTest3796 extends LinearOpMode {

    @Override
    public void runOpMode(){
        KowallskiCollectionServos3796 servo = new KowallskiCollectionServos3796(hardwareMap.crservo.get("collectionServoRight"), hardwareMap.crservo.get("collectionServoLeft"));
        waitForStart();

        while(opModeIsActive()){

            //Variables for the different buttons that will be pressed
            //It looks better than having to write gamepad1.left_stick_y every time or something
            boolean gamePad2AButton = gamepad2.a;
            boolean gamePad2BButton = gamepad2.b;
            telemetry.addLine("Test");
            if(gamePad2AButton)
            {
                telemetry.addLine("A");
                servo.collectOrEject(1.0);
            }
            else if(gamePad2BButton)
            {
                telemetry.addLine("B");
                servo.collectOrEject(-1.0);
            }else {
                telemetry.addLine("End");
                servo.stopMovement();
            }
            telemetry.update();

        }
    }
}
