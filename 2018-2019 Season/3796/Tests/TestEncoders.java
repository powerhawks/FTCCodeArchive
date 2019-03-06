package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.MotorSide;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;

@Autonomous(name="Encoder Test", group="Test")
@Disabled
public class TestEncoders extends LinearOpMode {
    //KowallskiMecanumDrive3796 drive;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftBack;
    static final double     TICKS_PER_INCH_L          = (5122.0/120);
    static final double     TICKS_PER_INCH_R          = (5133.0/120);
    static final double     DISTANCE_BETWEEN_WHEELS   =  11.40;
    public int inchesToTicks(double inches, MotorSide side)
    {

        //inches -= DISTANCE_BETWEEN_WHEELS;
        switch(side)
        {
            case LeftMotor:
                return (int)((inches * TICKS_PER_INCH_L));
            case RightMotor:
                return (int)((inches * TICKS_PER_INCH_R));
            default:
                return -1;
        }

    }
    public double ticksToInches(int ticks, MotorSide side)
    {
        double t = (double)ticks;
        switch(side)
        {
            case LeftMotor:
                return (int)((t / TICKS_PER_INCH_L));
            case RightMotor:
                return (int)((t / TICKS_PER_INCH_R));
            default:
                return -1;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = hardwareMap.dcMotor.get("leftBackDrive");
        rightFront = hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = hardwareMap.dcMotor.get("rightBackDrive");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        boolean FirstTime = true;
  //      drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
        while(opModeIsActive()) {
            if(FirstTime)
            {
                telemetry.addLine("Starting...");
                telemetry.update();

                leftBack.setTargetPosition(inchesToTicks(48, MotorSide.LeftMotor));
                rightBack.setTargetPosition(inchesToTicks(48, MotorSide.RightMotor));

                leftBack.setPower(0.25);
                rightBack.setPower(0.25);
                leftFront.setPower(0.25);
                rightFront.setPower(0.25);

                while(opModeIsActive() && (rightBack.isBusy() && leftBack.isBusy()))
                {
                    telemetry.addLine("Left Status: " + ticksToInches(leftBack.getCurrentPosition(), MotorSide.LeftMotor) + " / " + 120);
                    telemetry.addLine("Right Status: " + ticksToInches(rightBack.getCurrentPosition(), MotorSide.RightMotor) + " / " + 120);
                    telemetry.update();
                }

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                FirstTime = false;
            }
        }
    }
}
