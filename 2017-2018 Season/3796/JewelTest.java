package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Created by Power Hawks Robotics on 12/23/2017.
 */


@Autonomous(name = "Jewel Autonomous Test", group = "tests")
public class JewelTest extends LinearOpMode {
    //This is a autonomous opmode to test just our ability to knock the correct jewel off

    @Override
    public void runOpMode() {
        Side3796 side = Side3796.Red1;

        JAWLDrive3796 drive = new JAWLDrive3796(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        drive.setEncoders(true);
        JAWLColorArm3796 colorArm = new JAWLColorArm3796(hardwareMap.servo.get("color_arm"));
        JAWLColorSensor3796 colorDistanceSensor = new JAWLColorSensor3796(hardwareMap.get(ColorSensor.class, "color_distance"), hardwareMap.get(DistanceSensor.class, "color_distance"));
        colorArm.armUp();
        String color;

        waitForStart();

        /*if(opModeIsActive()) {
            colorArm.armDown();
            sleep(1000);
            String color = colorDistanceSensor.getColor();
            telemetry.addData("Color", color);
            telemetry.update();
            sleep(1000);
            if (color.equals("RED")) {
                //knock off the front jewel
                drive.forward(200, 1);
                //Wait until done
                sleep(1000);
                //Return to original position
                drive.backwards(200,1);
            } else if (color.equals("BLUE")) {
                //Knock off the back jewel
                drive.backwards(200,1);
                //Wait until done
                sleep(1000);
                //Return to original position
                drive.forward(200,1);
            } else if (color.equals("UR BAD")) {
                telemetry.addLine("COLOR NOT FOUND");
                telemetry.update();
            }

            colorArm.armUp();

            telemetry.addLine("Op mode finished");
            telemetry.update();
            }
        }*/

        if (opModeIsActive()) {
            colorArm.armDown();
            sleep(1000);
            color = colorDistanceSensor.getColor();
            telemetry.addData("Color", color);
            telemetry.update();
            sleep(1000);

            if (color.equals("BLUE")) {
                //If we are on platform Red1 and we scanned the ball to be red, we want to knock the front ball
                drive.turnRight(400, 1);
                sleep(1900);
                colorArm.armUp();
                sleep(500);
                drive.turnRight(-400, 1);
                sleep(500);
            } else if (color.equals("RED")) {
                //If we are on platform Red1 and we scanned the ball to be blue, we want to knock the back ball
                drive.turnRight(-400, 1);
                sleep(1900);
                colorArm.armUp();
                sleep(500);
                drive.turnRight(400, 1);
                sleep(500);
            } else if (color.equals("UR BAD")) {
                telemetry.addLine("COLOR NOT FOUND");
                telemetry.update();
            }

        }
    }
}
