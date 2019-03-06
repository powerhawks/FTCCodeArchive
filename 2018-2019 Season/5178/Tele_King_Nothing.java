package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by King Nothing
 */

@TeleOp(name = "Tele_King_Nothing", group = "Teleop")
public class Tele_King_Nothing extends Skele_Op {

    @Override
    public void runOpMode(){
        initRobot();

        waitForStart();

        while(opModeIsActive()){



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////The controls for the robot //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            if (gamepad1.left_stick_y > .05 || gamepad1.left_stick_y < -.05){
                LeftDrive.setPower(gamepad1.left_stick_y * 1.0);
            }
            if (gamepad1.right_stick_y > .05 || gamepad1.right_stick_y < -.05){
                RightDrive.setPower(gamepad1.right_stick_y * -1.0);
            }


            /*
            if(gamepad1.right_bumper){
                RightDrive.setPower(1);
            }

            else if (!gamepad1.right_bumper){
                RightDrive.setPower(0);

            }   if (gamepad1.left_bumper){
                LeftDrive.setPower(-1);
            }

            else if (!gamepad1.left_bumper){
               LeftDrive.setPower(0);
            }
            */
            if(gamepad1.a){
                LeftDrive.setPower(1);
                RightDrive.setPower(-1);
            }

            else if(!gamepad1.a){
                LeftDrive.setPower(0);
                RightDrive.setPower(0);
            }
            if(gamepad1.y){
                LeftDrive.setPower(-1);
                RightDrive.setPower(1);
            }
            else if(!gamepad1.y){
                LeftDrive.setPower(0);
                RightDrive.setPower(0);
            }



        }
    }

}
