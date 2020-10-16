package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name = "Motor Test", group = "Tests")
public class MotorTest extends LinearOpMode {
	DcMotorEx motor;
	@Override
	public void runOpMode(){
		motor = hardwareMap.get(DcMotorEx.class, "motor");
		waitForStart();
		while (opModeIsActive()){
			telemetry.addData("Power", motor.getPower());
			telemetry.addData("Velocity", motor.getVelocity());
			telemetry.addData("Position", motor.getCurrentPosition());
			telemetry.update();
			motor.setPower(gamepad1.left_stick_y);
		}
	}
}
