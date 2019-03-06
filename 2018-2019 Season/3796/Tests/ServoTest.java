package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * November 1, 2018
 * Chase Galey and Lincoln Doney
 * A servo test to show the build team how fast they can spin
 */

@TeleOp(name = "Servo Test", group = "X")
@Disabled
public class ServoTest extends LinearOpMode {
    CRServo servo;
    CRServo servo2;
    public void runOpMode() throws InterruptedException
    {
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()) {
            servo = hardwareMap.get(CRServo.class, "servo_1");
            servo2 = hardwareMap.get(CRServo.class, "servo_2");
            telemetry.addData(">", "Press Stop to end test." );
            //Stops the servo
            if(gamepad1.a)
            {
                telemetry.addData("Servo Position", "STOPPING SERVO");
                servo.setPower(0.0);
                servo2.setPower(0.0);
            }
            //Sets the servo to turn backwards at full speed
            else if(gamepad1.left_bumper)
            {
                telemetry.addData("Servo Position", "Moving Left...");
                servo.setPower(-1.0);
                servo2.setPower(-1.0);
            }else if(gamepad1.right_bumper)
            {
                telemetry.addData("Servo Position", "Moving Right...");
                servo.setPower(1.0);
                servo2.setPower(1.0);
            }
            else
            {
                servo.setPower(0.0);
                servo2.setPower(0.0);
            }
            telemetry.addData(">", "Done");
            telemetry.update();

        }
    }
}
