package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by KILLER QUEEN HAS ALREADY TOUCHED THIS CODE on 11/27/2018.
 */
//NANI?!?!
@Autonomous (name = "AUTOBOTS_LSD_CRATER" , group="Autonomous")

public class AUTOBOTS_LSD_CRATER extends BaseOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();

//        HoldMarker();

        waitForStart();
//        HoldMarker();
//        collectionRotate.setPosition(1);//Resting
        boolean done = false;
        boolean running = true;
        //this.relicTrackables.activate();
        try {
            while (opModeIsActive()) {
                // Code Here

                CloseGate();
                ParkUp();

                //Lower and Unhook
                RobotDownInch(7);
                turnLeftDeg(15);
                backwardInch(3);
                turnRightDeg(15);
                //turnRightGyro(1,15);


                //drive to Depot
               // turnRightDeg(5);
                //backwardInch(15);
                //ParkDown();



                break;
            }
            Thread.sleep(1000);
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        } finally {
            StopMotors();
        }
    }
}
