package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Config.Config;

@TeleOp(name = "CfgGet", group = "X")
@Disabled
public class CfgTester extends LinearOpMode {


    @Override public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Name: " + Config.init());
            telemetry.update();
        }
    }
}