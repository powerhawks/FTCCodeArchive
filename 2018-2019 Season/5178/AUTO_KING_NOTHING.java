package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by King Nothing
 */
@Autonomous (name = "KING_NOTHING!", group = "Autonomous")
public class AUTO_KING_NOTHING extends BaseOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        try {
            while (opModeIsActive()) {

                //insert code here

                CloseGate();
                ParkUp();
                //Lower and Unhook(look is AUTOBOTS_LSD
                RobotDownInch(7);
                turnLeftDeg(15);
                backwardInch(3);
                turnRightDeg(15);
                backwardInch(15);
                sleep(200);
                //go to Depot
                forwardInch(7.5);
                turnLeftDeg(105);
                backwardInch(34);
                turnLeftDeg(52);
                backwardInch(30);
                OpenGate();
                forwardInch(50);









            }
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());

        } finally {
            StopMotors();
        }
    }
}
