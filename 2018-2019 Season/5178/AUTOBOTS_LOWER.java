package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by KILLER QUEEN HAS ALREADY TOUCHED THIS CODE on 11/27/2018.
 */
//NANI?!?!

@Autonomous(name = "AUTOBOTS_LOWER" , group="Autonomous")

public class AUTOBOTS_LOWER extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();

//        HoldMarker();
        waitForStart();
        boolean done = false;
        boolean running = true;
        //this.relicTrackables.activate();
        try {
            while (opModeIsActive()) {
                // Code Here

                //Lower and Unhook
                RobotDownInch(7);
                telemetry.addData("Finished Lowering", 1);
                turnLeftDeg(25);
                backwardInch(2);
                turnRightDeg(25);
                updateTelemetry(telemetry);
                //DONE



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
