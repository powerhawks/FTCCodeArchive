package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;

@TeleOp(name="Forward Test", group="Test")
@Disabled
public class TestForwardTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        KowallskiMecanumDrive3796 drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_y > 0.1)
            {
                drive.leftFrontWheel.setPower(-1.0);
                drive.leftBackWheel.setPower(-1.0);
                drive.rightFrontWheel.setPower(1.0);
                drive.rightBackWheel.setPower(1.0);
            }
            else if(gamepad1.left_stick_y < -0.1)
            {
                drive.leftFrontWheel.setPower(1.0);
                drive.leftBackWheel.setPower(1.0);
                drive.rightFrontWheel.setPower(-1.0);
                drive.rightBackWheel.setPower(-1.0);
            }
            else
            {
                drive.leftFrontWheel.setPower(0);
                drive.leftBackWheel.setPower(0);
                drive.rightFrontWheel.setPower(0);
                drive.rightBackWheel.setPower(0);
            }
        }
    }
}
