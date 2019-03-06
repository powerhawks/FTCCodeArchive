package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.MotorSide;

@TeleOp(name = "Encoder Recorder")
public class EncoderRecorder extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftBack;
    static final double     TICKS_PER_INCH_L          = (5122.0/560.0);
    static final double     TICKS_PER_INCH_R          = (5133.0/560.0);
    public int inchesToTicks(double inches, MotorSide side)
    {
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
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        while(opModeIsActive()) {
            telemetry.addLine("Left: " + leftBack.getCurrentPosition() + " ticks");
            telemetry.addLine("Right: " + rightBack.getCurrentPosition() + " ticks");
            telemetry.update();
        }
    }
}