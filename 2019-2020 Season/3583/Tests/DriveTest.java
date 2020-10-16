package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Gyro;
import org.firstinspires.ftc.teamcode.TeleSkeletonSkystone;

import java.util.Arrays;
@Disabled
@TeleOp(name = "Drive Test")
public class DriveTest extends TeleSkeletonSkystone {
    private static final double ACCEPTINPUTTHRESHOLD = 0.15;
    private static final double LOWPOWERMULTIPLIER = 0.25;
    private static final double MIDPOWERMULTIPLIER = 0.75;
    private static final double FULLPOWERMULTIPLIER = 1;
    private DriveType drive = DriveType.ARCADE;
    private DriveSpeed speed = DriveSpeed.LOWPOWER;
    private final double SCALAR = 1/.7071067811865475;
    private boolean field = false;
    @Override
    public void runOpMode() {
        initRobot();
        initTeleGyro();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speed = DriveSpeed.MIDPOWER;
            } else if (gamepad1.right_bumper) {
                speed = DriveSpeed.HIGHPOWER;
            } else {
                speed = DriveSpeed.LOWPOWER;
            }
            if (drive == DriveType.ARCADE) {
                if (field) {
                    telemetry.addLine("Drive System: Arcade Field-Centric");
                } else {
                    telemetry.addLine("Drive System: Arcade Robot-Centric");

                }
            } else if (drive == DriveType.WEB) {
                if (field) {
                    telemetry.addLine("Drive System: Web Field-Centric");
                } else {
                    telemetry.addLine("Drive System: Web Robot-Centric");
                }
            } else if (drive == DriveType.ARCADE_SIMPLE) {
                if (field) {
                    telemetry.addLine("Drive System: Simple Arcade Field-Centric");
                } else {
                    telemetry.addLine("Drive System: Simple Arcade Robot-Centric");
                }
            }
            telemetry.addData("Left Front Power", leftFrontWheel.getPower());
            telemetry.addData("Left Back Power", leftBackWheel.getPower());
            telemetry.addData("Right Front Power", rightFrontWheel.getPower());
            telemetry.addData("Right Back Power", rightBackWheel.getPower());

            if (gamepad1.dpad_up) {
                drive = DriveType.ARCADE;
            } else if (gamepad1.dpad_down) {
                drive = DriveType.ARCADE_SIMPLE;
            } else if (gamepad1.dpad_left) {
                drive = DriveType.WEB;
            }
            if (gamepad1.x) {
                field = false;
            } else if (gamepad1.b) {
                field = true;
            }
            if (gamepad1.y) {
                initTeleGyro();
                OFFSET = 0;
            }

            double inputX, inputY, inputC;
            inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
            inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
            inputC = Math.abs(gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x : 0;
            if (drive == DriveType.ARCADE) {
                if (field) {
                    if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    fieldCentric((-MIDPOWERMULTIPLIER * inputY), (-MIDPOWERMULTIPLIER * inputX), (-MIDPOWERMULTIPLIER * inputC));
                } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    fieldCentric((-FULLPOWERMULTIPLIER * inputY), (-FULLPOWERMULTIPLIER * inputX), (-FULLPOWERMULTIPLIER * inputC));
                } else {
                    fieldCentric((-LOWPOWERMULTIPLIER * inputY), (-LOWPOWERMULTIPLIER * inputX), (-LOWPOWERMULTIPLIER * inputC));
                }
            } else {
                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    arcadeMecanum((-MIDPOWERMULTIPLIER * inputY), (MIDPOWERMULTIPLIER * inputX), (-MIDPOWERMULTIPLIER * inputC));
                } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    arcadeMecanum((-FULLPOWERMULTIPLIER * inputY), (FULLPOWERMULTIPLIER * inputX), (-FULLPOWERMULTIPLIER * inputC));
                } else {
                    arcadeMecanum((-LOWPOWERMULTIPLIER * inputY), (LOWPOWERMULTIPLIER * inputX), (-LOWPOWERMULTIPLIER * inputC));
                }
            }
        }else if(drive == DriveType.WEB){
                if(field){
                    fieldCentricWeb((-inputY), (inputX), (-inputC));
                }else{
                    webDrive((inputY), (-inputX), (-inputC));
                }
            }else if(drive == DriveType.ARCADE_SIMPLE){
                if(field){
                    fieldCentricArcadeSimple(inputY, inputX, inputC);
                }else{
                    arcadeMecanumSimplified(-inputY, inputX, -inputC);
                }
            }
            telemetry.update();
        }
    }

    private void webDrive(double y, double x, double c){
        //,7071067811865475

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        double angle = Math.atan2(y, x);
        double magnitude = Math.sqrt((Math.pow(y, 2) + Math.pow(x, 2)));
        double rightFrontVal = (speed.getSpeed() * SCALAR * ((Math.sin(angle + .25*Math.PI) * magnitude) + c));
        double leftBackVal = (speed.getSpeed() * SCALAR * (Math.sin(angle + .25*Math.PI) * magnitude) - c);
        double leftFrontVal = (speed.getSpeed() * SCALAR * (Math.sin(angle - .25*Math.PI) * magnitude) - c);
        double rightBackVal = (speed.getSpeed() * SCALAR * (Math.sin(angle - .25*Math.PI) * magnitude) + c);
        telemetry.addData("Left Front Val", leftFrontVal);
        telemetry.addData("Left Back Val", leftBackVal);
        telemetry.addData("Right Front Val", rightFrontVal);
        telemetry.addData("Right Back Val", rightBackVal);
        telemetry.update();

        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFrontWheel.setPower(leftFrontVal);
        rightFrontWheel.setPower(rightFrontVal);
        leftBackWheel.setPower(leftBackVal);
        rightBackWheel.setPower(rightBackVal);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void fieldCentricWeb(double y, double x, double c){
        double forward = -y;
        double strafe = x;
        angles   = Gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyroDegrees = -angles.firstAngle + OFFSET;//make this negative
        double gyroRadians = gyroDegrees * (Math.PI/180);
        double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
        strafe = (-forward * Math.sin(gyroRadians)) + (strafe * Math.cos(gyroRadians));
        forward = temp;
        webDrive(forward, -strafe, c);
    }

    void arcadeMecanumSimplified(double y, double x, double c) {

        double leftFrontVal = y - x + c;
        double rightFrontVal = -y - x + c;

        // try setting these equal to their front most counterpart
        double leftBackVal = y + x + c;
        double rightBackVal = -y + x + c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        //double scaledPower = gearShift[gear];
        leftFrontWheel.setPower(leftFrontVal * speed.getSpeed());
        rightFrontWheel.setPower(rightFrontVal * speed.getSpeed());
        leftBackWheel.setPower(leftBackVal * speed.getSpeed());
        rightBackWheel.setPower(rightBackVal * speed.getSpeed());
    }

    private void fieldCentricArcadeSimple(double y, double x, double c){
        double forward = -y;
        double strafe = x;
        angles   = Gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyroDegrees = -angles.firstAngle + OFFSET;//make this negative
        double gyroRadians = gyroDegrees * (Math.PI/180);
        double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
        strafe = (-forward * Math.sin(gyroRadians)) + (strafe * Math.cos(gyroRadians));
        forward = temp;
        arcadeMecanumSimplified(forward, -strafe, c);
    }
}

