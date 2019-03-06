package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by KILLER QUEEN HAS ALREADY TOUCHED THIS CODE on 11/27/2018.
 */
//NANI?!?!

@Autonomous (name = "AUTOBOTS_LSD_NO_CRASHING" , group="Autonomous")

public class AUTOBOTS_LSD_NO_CRASHING extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();

//        HoldMarker();
        telemetry.addData("Mark ", 1);
        updateTelemetry(telemetry);

        waitForStart();
        boolean done = false;
        boolean running = true;
        telemetry.addData("Mark ", 2);
        updateTelemetry(telemetry);

        //this.relicTrackables.activate();
        try {
            while (opModeIsActive()) {

                telemetry.addData("Mark ", 3);
                updateTelemetry(telemetry);

                CloseGate();
                ParkUp();


                //Lower and Unhook
                RobotDownInch(7);

                //turnLeftGyro(1,15);
                turnLeftDeg(15);
                backwardInch(3);
                turnRightDeg(20);
                //turnRightGyro(1,15);


                //drive to Depot
                backwardInch(32);//30
                //                DeployMarker();
                OpenGate();

                turnRightDeg(38);//42.55
                //turnRightGyro(1,45);
                forwardInch(50);
                //ParkDown();

                telemetry.addData("Mark ", 4);
                updateTelemetry(telemetry);


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


