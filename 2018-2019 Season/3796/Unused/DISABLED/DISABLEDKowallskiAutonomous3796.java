package org.firstinspires.ftc.teamcode.Unused.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.KowallskiSide3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMecanumDrive3796;
import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiMineralArm3796;

@Disabled
//Not Being Used
public class DISABLEDKowallskiAutonomous3796 extends LinearOpMode {

    KowallskiSide3796 side;

    @Override
    public void runOpMode() throws InterruptedException {

        KowallskiMecanumDrive3796 drive = new KowallskiMecanumDrive3796(hardwareMap.dcMotor.get("rightFrontDrive"), hardwareMap.dcMotor.get("rightBackDrive"), hardwareMap.dcMotor.get("leftFrontDrive"), hardwareMap.dcMotor.get("leftBackDrive"));
        KowallskiMineralArm3796 arm = new KowallskiMineralArm3796(hardwareMap.dcMotor.get("collectionArmRight"), hardwareMap.dcMotor.get("collectionArmLeft"));
//        KowallskiCollectionServos3796 servo = new KowallskiCollectionServos3796(hardwareMap.crservo.get("collectionServoRight"), hardwareMap.crservo.get("collectionServoLeft"));
//        KowallskiRamp3796 ramp = new KowallskiRamp3796(hardwareMap.dcMotor.get("rampMotor"));
        double POWER = 1.0;

        waitForStart();

        while(opModeIsActive()){


            //drive.setEncoders(true);

            arm.moveUpOrDown(POWER);

            sleep(500);

            arm.stopMovement();
            sleep(100);
            drive.leftRight(POWER);

            sleep(500);

            drive.stopMovement();

            /*
            sleep(100);
            drive.forwardBackward(POWER);

            sleep(1500);

            if(side == KowallskiSide3796.redDepot){

                drive.stopMovement();
                sleep(100);
                drive.leftRight(POWER);

                sleep(1000);

                drive.stopMovement();
                sleep(100);
                drive.turn(-POWER);

                sleep(350);

                drive.stopMovement();
                sleep(100);
                drive.leftRight(POWER);

                sleep(500);

                drive.stopMovement();
                sleep(100);
                drive.forwardBackward(POWER);

                sleep(3000);

                drive.stopMovement();

            }
*/
        }

    }
}
