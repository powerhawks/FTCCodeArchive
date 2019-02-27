package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Created by Chase Galey and Joe Lewis
 * January 11, 2017
 * <p>
 * A bare minimum autonomous that only knocks the opposing jewel off
 */

@Autonomous(name = "Only Knock Jewel Red Team")
public class JAWLOnlyKnockJewel3976Red extends LinearOpMode {
    public void runOpMode() {
        Side3796 side = Side3796.Red1;

        JAWLDrive3796 drive = new JAWLDrive3796(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        drive.setEncoders(true);
        JAWLColorArm3796 colorArm = new JAWLColorArm3796(hardwareMap.servo.get("color_arm"));
        JAWLGrabber3796 grabber = new JAWLGrabber3796(hardwareMap.servo.get("left_arm"), hardwareMap.servo.get("right_arm"));
        JAWLColorSensor3796 colorDistanceSensor = new JAWLColorSensor3796(hardwareMap.get(ColorSensor.class, "color_distance"), hardwareMap.get(DistanceSensor.class, "color_distance"));
        colorArm.armUp();
        String color;

        waitForStart();

        if (opModeIsActive()) {

            grabber.leftArmClose();
            grabber.rightArmClose();

            colorArm.armDown();
            sleep(1000);
            color = colorDistanceSensor.getColor();
            telemetry.addData("Color", color);
            telemetry.update();
            sleep(1000);

            if (side == Side3796.Red1 || side == Side3796.Red2) {
                if (color.equals("RED")) {
                    //If we are on platform Red1 and we scanned the ball to be red, we want to knock the front ball
                    drive.turnRight(400, 1);
                    sleep(1900);
                    colorArm.armUp();
                    sleep(500);
                    drive.turnRight(-400, 1);
                    sleep(500);
                } else if (color.equals("BLUE")) {
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
            } else if (side == Side3796.Blue1 || side == Side3796.Blue2) {
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
}
