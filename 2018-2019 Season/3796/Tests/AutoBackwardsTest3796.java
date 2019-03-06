package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Enums.MotorSide;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;

@Autonomous(name="Back Test", group="Test")
@Disabled
public class AutoBackwardsTest3796 extends LinearOpMode {
    KowallskiMecanumDrive3796 drive;
    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    static final double AUTO_POWER = 0.25;
    static final double ROBOT_WIDTH = 17;
    static final double L_R_MOD = 8;

    static final double ROBOT_HEIGHT = 17;

    static final double TICKS_PER_INCH_L = (5122.0 / 120);
    static final double TICKS_PER_INCH_R = (5133.0 / 120);

    static final double ACCELERATION_VALUE = 0.01;
    static final double ACCELERATION_MAX = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));

        waitForStart();
        boolean FirstTime = true;
        while(opModeIsActive())
        {
            if(FirstTime) {
                drive.forwardBackward(-0.5);
                sleep(2500);
                drive.stopMovement();
                FirstTime = false;
            }

        }
    }
    public void DriveBackward(double inches) throws InterruptedException {
        drive.rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
        drive.leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        drive.leftBackWheel.setDirection(DcMotor.Direction.REVERSE);

        drive.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Backward...");
        telemetry.update();

        drive.leftBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.LeftMotor));
        drive.rightBackWheel.setTargetPosition(inchesToTicks(inches, MotorSide.RightMotor));

        drive.leftBackWheel.setPower(AUTO_POWER);
        drive.rightBackWheel.setPower(AUTO_POWER);
        drive.leftFrontWheel.setPower(AUTO_POWER);
        drive.rightFrontWheel.setPower(AUTO_POWER);

        while (opModeIsActive() && (drive.rightBackWheel.isBusy() && drive.leftBackWheel.isBusy())) {
            if(drive.leftFrontWheel.getPower() < ACCELERATION_MAX) {
                drive.leftFrontWheel.setPower(drive.leftFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.rightFrontWheel.setPower(drive.rightFrontWheel.getPower() + ACCELERATION_VALUE);
                drive.leftBackWheel.setPower(drive.leftBackWheel.getPower() + ACCELERATION_VALUE);
                drive.rightBackWheel.setPower(drive.rightBackWheel.getPower() + ACCELERATION_VALUE);
            }
            telemetry.addLine("Left Status: " + ticksToInches(drive.leftBackWheel.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
            telemetry.addLine("Right Status: " + ticksToInches(drive.rightBackWheel.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
            telemetry.update();
        }

        drive.leftBackWheel.setPower(0);
        drive.leftFrontWheel.setPower(0);
        drive.rightBackWheel.setPower(0);
        drive.rightFrontWheel.setPower(0);

        drive.leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        drive.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int inchesToTicks(double inches, MotorSide side) throws InterruptedException {

        switch (side) {
            case LeftMotor:
                return (int) ((inches * TICKS_PER_INCH_L));
            case RightMotor:
                return (int) ((inches * TICKS_PER_INCH_R));
            default:
                return -1;
        }

    }

    public double ticksToInches(int ticks, MotorSide side) throws InterruptedException {
        double t = (double) ticks;
        switch (side) {
            case LeftMotor:
                return (int) ((t / TICKS_PER_INCH_L));
            case RightMotor:
                return (int) ((t / TICKS_PER_INCH_R));
            default:
                return -1;
        }
    }

}
