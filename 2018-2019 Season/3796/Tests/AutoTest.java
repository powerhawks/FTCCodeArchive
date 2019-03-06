package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;

@Autonomous(name ="Autonomous Test", group = "X")
@Disabled
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws NullPointerException{
        KowallskiMecanumDrive3796 drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));

        waitForStart();


        try {
            drive.moveFrontRightWheel(1);
            telemetry.addLine("Right Front Found");
        }catch (NullPointerException n) {
            telemetry.addLine("Right Front Not Found");
        }
        telemetry.update();
        sleep(1000000000);

    }

}
