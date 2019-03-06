package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by King Everything
 */
public abstract class Skele_Op extends LinearOpMode {

    //This defines the motors the motors
    public DcMotor LeftDrive;
    public DcMotor RightDrive;

    public void initRobot(){
        //these define where they can show up on the phones
        RightDrive = hardwareMap.dcMotor.get("rDrive");
        LeftDrive = hardwareMap.dcMotor.get("lDrive");
        telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", gamepad1.right_stick_x);
        updateTelemetry(telemetry);

    }
}
