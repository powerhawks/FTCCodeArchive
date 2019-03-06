package org.firstinspires.ftc.teamcode.Unused.Farquad;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Joe is the big thot", group = "X")
@Disabled
public class FarquadAutononomous extends LinearOpMode {


    @Override
    public void runOpMode() {
        //Changed name from joe to leftDrive and thot to rightDrive
        DcMotor leftDrive, rightDrive;

        leftDrive = hardwareMap.dcMotor.get("lDrive");
        rightDrive = hardwareMap.dcMotor.get("rDrive");

        waitForStart();

        while (opModeIsActive()) {

            leftDrive.setPower(1);
            rightDrive.setPower(-1);

            telemetry.addData("leftDrive power", 1);
            telemetry.addData("rightDrive power", -1);
            telemetry.addLine("robot direction: forward");
            telemetry.update();

            sleep(2000);

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            telemetry.addData("leftDrive power", 0);
            telemetry.addData("rightDrive power", 0);
            telemetry.addLine("robot direction: none");
            telemetry.update();

            sleep(500);


            leftDrive.setPower(-1);
            rightDrive.setPower(-1);

            telemetry.addData("leftDrive power", -1);
            telemetry.addData("rightDrive power", -1);
            telemetry.addLine("robot direction: turning left");
            telemetry.update();

            sleep(750);


            leftDrive.setPower(0);
            rightDrive.setPower(0);

            telemetry.addData("leftDrive power", 0);
            telemetry.addData("rightDrive power", 0);
            telemetry.addLine("robot direction: none");
            telemetry.update();

            sleep(500);


            leftDrive.setPower(1);
            rightDrive.setPower(-1);

            telemetry.addData("leftDrive power", 1);
            telemetry.addData("rightDrive power", -1);
            telemetry.addLine("robot direction: forward");
            telemetry.update();

            sleep(3000);


            leftDrive.setPower(0);
            rightDrive.setPower(0);

            telemetry.addData("leftDrive power", 0);
            telemetry.addData("rightDrive power", 0);
            telemetry.addLine("robot direction: none");
            telemetry.addLine("Shutting down");
            telemetry.update();

            break;
        }
    }
}