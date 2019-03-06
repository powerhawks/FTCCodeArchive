package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tyler Lang of Clan Farquad on 11/10/2018.
 */

@TeleOp(name="SampleTest")
public class SampleTest extends LinearOpMode {

    ColorSensor sightBud;
    DcMotor RDrive;
    DcMotor LDrive;
    Servo Sampler;
    float hsvValues[] = {0F,0F,0F};



    // inch movement methods
    //wheel radius
    static double WRadius = 1.9375; // CHANGE THIS VALUE 1
    static double WCirc = (2 * 3.141592) * WRadius;
    // input:output, GR > 1 for torque, GR < 1 for speed
    static double GearRatio = 1;

    static int TicksPerRevolution = 1120; // CHANGE THIS VALUE 2

    // robot radius from turn point to wheels
    static double RRadius = 8.25; // CHANGE THIS VALUE 3
    static double RCirc = (2 * 3.141592) * RRadius;
    static int tolerance = 10;

    int ticksRight = 0;
    int ticksLeft = 0;

    @Override
    public void runOpMode() {
        RDrive = hardwareMap.dcMotor.get("RDrive");
        LDrive = hardwareMap.dcMotor.get("LDrive");
        Sampler = hardwareMap.servo.get("sample");

        sightBud = hardwareMap.colorSensor.get("sightBud");

        LDrive.setDirection(DcMotorSimple.Direction .REVERSE);

        RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        while(opModeIsActive()) {
            updateColorSensor();

            if (gamepad2.a){
                Sampler.setPosition(.218);
                sleep(1000);
                Sample();
                forwardInch(3.888889);
                Sampler.setPosition(.15);
                sleep(1000);
            }

            if (gamepad2.x){
                Sampler.setPosition(.218);
                sleep(1000);
                Sample();

                Sampler.setPosition(.15);
                sleep(1000);
            }

            if (gamepad2.b){
                Sampler.setPosition(.218);
                sleep(1000);
                logMotors();
                sleep(1000);
                Sampler.setPosition(.17);

            }



            //logMotors();
        }
    }
    public void Sample(){
        boolean SeeColor = false;
        boolean armDown = false;
        for (int i = 1; i <= 3; i++ ){
            if(isColor()){
                SeeColor = true;
            }
            forwardInch(1 / 1.66);//moves 1.66 inch

        }

        if (SeeColor){
            //put down arm    Tested value: .225
            Sampler.setPosition(.3);
            sleep(500);
            armDown = true;
        }
        forwardInch((12 / 3.66) * (1.2)); //moves 6 inch
        if (armDown){
            //put arm up
            Sampler.setPosition(.15);
            armDown = false;

        }

    }

    void updateColorSensor(){
        Color.RGBToHSV((sightBud.red() * 255) / 800, (sightBud.green() * 255) / 800,
                (sightBud.blue() * 255) / 800, hsvValues);
    }

    double Saturation(){
        return hsvValues[1];
    }

    boolean isGold(){
        updateColorSensor();
        float HUE = hsvValues[0];

        if (HUE < 65 && HUE > 20){
            return true;

        }
        else{
            return false;
        }
    }
    boolean isColor(){
        if (isGold() && Saturation() > .2 && Saturation() < .51){
            return true;
        }
        return false;
    }

    void forwardInch(double inch){
        double ticks = convertToTicks(inch);
        forwardPos((int)ticks);
    }

    double convertToTicks(double inch){

        return (inch / WCirc) * TicksPerRevolution * GearRatio;
    }

    void forwardPos(int ticks){
        int targetLDrive = LDrive.getCurrentPosition() + ticks;
        int targetRDrive = RDrive.getCurrentPosition() + ticks;

        LDrive.setTargetPosition(targetLDrive);
        RDrive.setTargetPosition(targetRDrive);
        LDrive.setPower(1);
        RDrive.setPower(1);

        try {
            boolean done = false;
            while (!done) {
                logMotors();
                if (!LDrive.isBusy() || !RDrive.isBusy()) {
                    done = true;
                }
                idle();
            }
        } finally {
            StopMotors();
        }
    }

    void StopMotors(){

        // all motors.setPower(0.0);
        LDrive.setPower(0.0);
        RDrive.setPower(0.0);
    }

    void logMotors() {
        telemetry.addData("LDrive", LDrive.getCurrentPosition());
        telemetry.addData("RDrive", RDrive.getCurrentPosition());
        telemetry.addData("Sampler", Sampler.getPosition());

        telemetry.addData("See Color?", isColor());
        telemetry.addData("Color HSV (HUE)", hsvValues[0]);
        telemetry.addData("Color HSV (Saturation)", hsvValues[1]);

        telemetry.update();
    }

}
