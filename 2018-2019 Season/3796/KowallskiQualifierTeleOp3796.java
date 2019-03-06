package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.diag;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiCollectionServos3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMineralArm3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiRamp3796;

/**
 * Name: TeleOp
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: November 17, 2018
 * */


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

/**
 * CONTROLLER SCHEME
 *
 ************************ Gamepad 1 - Chassis Driver - Will (STR + A):***************************
 *
 *                        LT                            RT
 *                    _.-'` `LB_                    _RB' `'-._
 *                 ,-' DU       `-.,____________,.-'    .-.   `-.
 *                /   .---.             ___            ( Y )     \
 *               /  ,' ,-. `.     SEL   /   \ STR   .-. `-` .-.   \
 *              / DL   |  | |DR  (__) | / \ | (__) ( X )   ( B )   \
 *             /    `. `-' ,'    __    \___/  RSU   `-` ,-. `-`     \
 *             |      `---`   ,-`LSU`-.      .---.     ( A )        |
 *             |       DD    / -'  `- \    ,'  .  `.    `-`         |
 *             |            |LSL    LSR| RSL -   - |RSR             |
 *             !             \ -.LSD- /    `.  '  ,'                |
 *             |              `-.__,-'       `---`                  |
 *             |                  _____________RSD                  |
 *             |             _,-'`                ``-._             |
 *             |          ,-'                          `-.          |
 *              \       ,'                                `.       /
 *               `.__,-'                                    `-.__,'
 *  Binding | Control                       | Full Binding Name
 *  -------------------------------------------------------------
 *  LSU     | Move robot up                 | Left Stick Up
 *  LSL     | Move robot left               | Left Stick Left
 *  LSR     | Move robot right              | Left Stick Right
 *  LSD     | Move robot down               | Left Stick Down
 *          |                               |
 *  RSU     | [UNBOUND]                     | Right Stick Up
 *  RSL     | Turn robot left               | Right Stick Left
 *  RSR     | Turn robot right              | Right Stick Right
 *  RSD     | [UNBOUND]                     | Right Stick Down
 *          |                               |
 *  LB      | Enable Strafing mode          | Left Bumper
 *  RB      | [UNBOUND]                     | Right Bumper
 *  RT      | [UNBOUND]                     | Right Trigger
 *  LT      | [UNBOUND]                     | Left Trigger
 *          |                               |
 *  X       | Enable Strafing mode          | X Button
 *  Y       | Enable Strafing mode          | Y Button
 *  B       | Enable Strafing mode          | B Button
 *  A       | Enable Strafing mode          | A Button
 *          |                               |
 *  DU      | [UNBOUND]                     | D-Pad Up
 *  DL      | [UNBOUND]                     | D-Pad Left
 *  DR      | [UNBOUND]                     | D-Pad Right
 *  DD      | [UNBOUND]                     | D-Pad Down
 *          |                               |
 *  DU + DL | Drive Diagonal Forward/Left   | D-Pad Up + Left
 *  DD + DL | Drive Diagonal Backward/Left  | D-Pad Down + Left
 *  DU + DR | Drive Diagonal Forward/Right  | D-Pad Up + Right
 *  DD + DR | Drive Diagonal Backward/Right | D-Pad Down + Right
 *          |                               |
 *  STR     | [UNBOUND]                     | Start Button
 *  SEL     | [UNBOUND]                     | Select Button
 *
 ****************** Gamepad 2- Mechanism Driver - Chase (STR + B) **********************
 *
 *                        LT                            RT
 *                    _.-'` `LB_                    _RB' `'-._
 *                 ,-' DU       `-.,____________,.-'    .-.   `-.
 *                /   .---.             ___            ( Y )     \
 *               /  ,' ,-. `.     SEL   /   \ STR   .-. `-` .-.   \
 *              / DL   |  | |DR  (__) | / \ | (__) ( X )   ( B )   \
 *             /    `. `-' ,'    __    \___/  RSU   `-` ,-. `-`     \
 *             |      `---`   ,-`LSU`-.      .---.     ( A )        |
 *             |       DD    / -'  `- \    ,'  .  `.    `-`         |
 *             |            |LSL    LSR| RSL -   - |RSR             |
 *             !             \ -.LSD- /    `.  '  ,'                |
 *             |              `-.__,-'       `---`                  |
 *             |                  _____________RSD                  |
 *             |             _,-'`                ``-._             |
 *             |          ,-'                          `-.          |
 *              \       ,'                                `.       /
 *               `.__,-'                                    `-.__,'
 *  Binding | Control                       | Full Binding Name
 *  -------------------------------------------------------------
 *  LSU     | Lower Ramp                    | Left Stick Up
 *  LSL     | [UNBOUND]                     | Left Stick Left
 *  LSR     | [UNBOUND]                     | Left Stick Right
 *  LSD     | Lift Ramp                     | Left Stick Down
 *          |                               |
 *  RSU     | Retract Ramp                  | Right Stick Up
 *  RSL     | [UNBOUND]                     | Right Stick Left
 *  RSR     | [UNBOUND]                     | Right Stick Right
 *  RSD     | Extend Ramp                   | Right Stick Down
 *          |                               |
 *  LB      | [UNBOUND]                     | Left Bumper
 *  RB      | [UNBOUND]                     | Right Bumper
 *  RT      | Collect Mineral               | Right Trigger
 *  LT      | Drop Mineral                  | Left Trigger
 *          |                               |
 *  X       | Raise Arm Quickly             | X Button
 *  Y       | Lower Arm Quickly             | Y Button
 *  B       | Raise Arm                     | B Button
 *  A       | Lower Arm                     | A Button
 *          |                               |
 *  DU      | [UNBOUND]                     | D-Pad Up
 *  DL      | [UNBOUND]                     | D-Pad Left
 *  DR      | [UNBOUND]                     | D-Pad Right
 *  DD      | [UNBOUND]                     | D-Pad Down
 *          |                               |
 *  DU + DL | [UNBOUND]                     | D-Pad Up + Left
 *  DD + DL | [UNBOUND]                     | D-Pad Down + Left
 *  DU + DR | [UNBOUND]                     | D-Pad Up + Right
 *  DD + DR | [UNBOUND]                     | D-Pad Down + Right
 *          |                               |
 *  STR     | [UNBOUND]                     | Start Button
 *  SEL     | [UNBOUND]                     | Select Button
 *
 */
@TeleOp (name="TeleOp", group = "Final")
public class KowallskiQualifierTeleOp3796 extends LinearOpMode {

    //Controller sensitivity
    private static final double SENSITIVITY = 0.9;


    @Override
    public void runOpMode(){

        //Initialize the mechanisms
        KowallskiMecanumDrive3796 drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
        KowallskiMineralArm3796 arm = new KowallskiMineralArm3796(hardwareMap.dcMotor.get("collectionArmRight"), hardwareMap.dcMotor.get("collectionArmLeft"));
        KowallskiCollectionServos3796 servo = new KowallskiCollectionServos3796(hardwareMap.crservo.get("collectionServoRight"), hardwareMap.crservo.get("collectionServoLeft"));
        KowallskiRamp3796 ramp = new KowallskiRamp3796(hardwareMap.dcMotor.get("rampMotor"), hardwareMap.dcMotor.get("rampExtender"));

        waitForStart();

        //Reset the motor of the ramp's encoders to zero
        ramp.resetRampMotor();

        while(opModeIsActive()){
            //Variables for the different buttons that will be pressed
            //It looks better than having to write gamepad1.left_stick_y every time or something
            double gamePad2LeftStickY = -gamepad2.left_stick_y;
            double gamePad2RightStickY = -gamepad2.right_stick_y;
            double gamePad2RightTrigger = gamepad2.right_trigger;
            double gamePad2LeftTrigger = gamepad2.left_trigger;
            boolean gamePad1AButton = gamepad1.a;
            boolean gamePad1BButton = gamepad1.b;
            boolean gamePad1XButton = gamepad1.x;
            boolean gamePad1YButton = gamepad1.y;
            boolean gamePad2AButton = gamepad2.a;
            boolean gamePad2BButton = gamepad2.b;
            boolean gamePad2XButton = gamepad2.x;
            boolean gamePad2YButton = gamepad2.y;
            boolean gamePad1DPadUp = gamepad1.dpad_up;
            boolean gamePad1DPadDown = gamepad1.dpad_down;
            boolean gamePad1DPadLeft = gamepad1.dpad_left;
            boolean gamePad1DPadRight = gamepad1.dpad_right;
            float gamePad1LeftStickY = -gamepad1.left_stick_y;
            float gamePad1LeftStickX = gamepad1.left_stick_x;
            float gamePad1RightStickX = gamepad1.right_stick_x;
            boolean gamePad1LeftBumper = gamepad1.left_bumper;

            //Boolean for strafing mode
            boolean mechanumMod = false;
            if(gamePad1XButton || gamePad1YButton || gamePad1AButton || gamePad1BButton || gamePad1LeftBumper)
                mechanumMod = true;

            //This next group of lines will ensure that nothing will be moving when we do not want it to.
            if(gamePad1LeftStickX == 0  && gamePad1RightStickX == 0 && gamePad1LeftStickY == 0 && !(gamePad1DPadDown && gamePad1DPadLeft && gamePad1DPadRight && gamePad1DPadUp)) {
                drive.stopMovement();
            }
            if(gamePad2LeftStickY == 0) {
                ramp.stopMovementRmp();
            }
            if(gamePad2RightStickY == 0) {
                ramp.stopMovementExt();
            }
            if(gamePad2LeftTrigger == 0 && gamePad2RightTrigger == 0) {
                servo.stopMovement();
            }
            if(!gamePad2BButton && !gamePad2AButton && !gamePad2XButton && !gamePad2YButton) {
                arm.stopMovement();
            }

            //Ramp Lifting or Lowering
            if(gamePad2LeftStickY > 0 || gamePad2LeftStickY < 0){
                ramp.liftOrLower((gamePad2LeftStickY));
            }
            //Ramp extension or contraction
            if(gamePad2RightStickY > 0 || gamePad2RightStickY < 0){
                ramp.extendOrContract((gamePad2RightStickY));
                telemetry.addLine("Current pos of extender: " + ramp.getTicks());
            }

            //Collection and Ejecting
            if(gamePad2LeftTrigger > 0){
                servo.collectOrEject(gamePad2LeftTrigger);
                telemetry.addLine("Collectiong...");
            }
            if(gamePad2RightTrigger > 0) {
                servo.collectOrEject(-gamePad2RightTrigger);
                telemetry.addLine("Ejecting...");
            }

            //Arm Controls
            if(gamePad2AButton){
                arm.moveUpOrDown(0.75);
            }
            if (gamePad2BButton) {
                arm.moveUpOrDown(-0.75);
            }
            if(gamePad2YButton){
                arm.moveUpOrDown(1);
            }
            if(gamePad2XButton){
                arm.moveUpOrDown(-1);
            }

            //DPad controls for Diagonals
            if(gamePad1DPadUp && gamePad1DPadRight){
                drive.diagonal(diag.upright);
            }else if(gamePad1DPadDown && gamePad1DPadRight){
                drive.diagonal(diag.downright);
            }else if(gamePad1DPadDown && gamePad1DPadLeft){
                drive.diagonal(diag.downleft);
            }else if(gamePad1DPadUp && gamePad1DPadLeft){
                drive.diagonal(diag.upleft);
            }

            Telemetry.Line one = telemetry.addLine("No Directional Movement");

            //Forward and backwards movement
            if(!mechanumMod && gamePad1LeftStickY > 0){
                telemetry.removeLine(one);
                telemetry.addLine("Moving Forward");
                drive.forwardBackward(gamePad1LeftStickY * SENSITIVITY);
            }else if(!mechanumMod && gamePad1LeftStickY < 0){
                telemetry.removeLine(one);
                telemetry.addLine("Moving Backwards");
                drive.forwardBackward(gamePad1LeftStickY * SENSITIVITY);
            }else if(mechanumMod && gamePad1LeftStickX > 0){
                telemetry.removeLine(one);
                telemetry.addLine("Moving Right");
                drive.leftRight(gamePad1LeftStickX * SENSITIVITY);
            }else if(mechanumMod && gamePad1LeftStickX < 0){
                telemetry.removeLine(one);
                telemetry.addLine("Moving Left");
                drive.leftRight(gamePad1LeftStickX * SENSITIVITY);
            }

            Telemetry.Line two = telemetry.addLine("No Rotational Movement");

            //Rotational Movement
            if(gamePad1RightStickX > 0){
                telemetry.removeLine(two);
                telemetry.addLine("Turning Right");
                drive.turn(gamePad1RightStickX);
            }else if(gamePad1RightStickX < 0){
                telemetry.removeLine(two);
                telemetry.addLine("Turning Left");
                drive.turn(gamePad1RightStickX);
            }

            telemetry.update();

        }
    }
}
