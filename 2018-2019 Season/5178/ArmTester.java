package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tyler Lang is FRICKEN GAY on 1/22/2019.
 */
@TeleOp(name="Arm Tester")
public class ArmTester extends LinearOpMode {

    Servo Rotate;
    CRServo Wheel;
    Servo Gate;

    double RotatePos = 1;

    @Override
    public void runOpMode() {
        Rotate = hardwareMap.servo.get("rotate");
        Wheel = hardwareMap.crservo.get("collectionW");
        Gate = hardwareMap.servo.get("gate");


        waitForStart();
        while(opModeIsActive()) {

            ///////////////////////////Rotate Code//////////////////////////////////////////////////
            if (gamepad2.left_stick_y > .1 && RotatePos < 1){
                RotatePos += .00001;
            }
            else if (gamepad2.left_stick_y < -.1 && RotatePos > 0){
                RotatePos -= .00001;
            }
            if (gamepad2.left_stick_button){
                RotatePos = .05;
            }
            Rotate.setPosition(RotatePos);


            ///////////////////////////Wheel Code///////////////////////////////////////////////////
            if (gamepad2.a) {
                Wheel.setPower(-0.8);
                //intake
            } else if (gamepad2.b) {
                //outtake
                Wheel.setPower(0.8);

            } else if (!gamepad2.a && !gamepad2.b) {
                Wheel.setPower(0);
                //neutralize wheel intake
            }

            ///////////////////////////Gate Code////////////////////////////////////////////////////
            if(RotatePos > .9){
                OpenGate();
            }
            else{
                CloseGate();
            }






        }
    }
    public void CloseGate(){
        Gate.setPosition(.85);
    }

    public void OpenGate(){
        Gate.setPosition(.52);
    }
}
