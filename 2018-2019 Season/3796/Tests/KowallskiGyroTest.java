package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;

/**
 *Made on December 9, 2018
 *Chase Galey and Lincoln Doney
 *A simple autonomous code to Let the Robot "land"
*/

@Autonomous(name = "Test", group = "X")
@Disabled
public class KowallskiGyroTest extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ModernRoboticsI2cGyro  gyro    = null;
    ElapsedTime timer = new ElapsedTime();
    KowallskiMecanumDrive3796 drive;
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();

        //Just a way for me to make sure that the code will only run once while still keeping the same code structure.
        int firstTime = 1;

        while(opModeIsActive() && firstTime == 1){
        }
    }

    void setup()
    {
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        drive = new KowallskiMecanumDrive3796(
                hardwareMap.dcMotor.get("rightFrontDrive"),
                hardwareMap.dcMotor.get("rightBackDrive"),
                hardwareMap.dcMotor.get("leftFrontDrive"),
                hardwareMap.dcMotor.get("leftBackDrive"));
    }
}
