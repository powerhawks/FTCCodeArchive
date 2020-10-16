package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Test")
public class LiftTest extends TeleSkeletonSkystone{
	@Override
	public void runOpMode(){
		initRobot();
		waitForStart();
		while (opModeIsActive()) {
			telemetry.addData("Brake", leftLift.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE);
			telemetry.update();
			leftLift.setPower(gamepad1.left_stick_y);
			rightLift.setPower(gamepad1.right_stick_y);
			if (gamepad1.left_bumper) {
				leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			} else if (gamepad1.right_bumper) {
				leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			}
		}
	}
}
