package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Power Hawks Robotics on 1/2/2018.
 */
@Autonomous(name="Test_Autonomous_Encoders", group="Autonomous")

public class Test_Autonomous_Encoders extends BaseOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        this.relicTrackables.activate();
        try {
            while (opModeIsActive()) {
                forwardInch(12);
                turnLeftDeg(90);
                break;
            }
        } catch (Exception e) {
        } finally {
            StopMotors();
        }
        idle();
    }

    //    }
    long prevTime = System.currentTimeMillis();
    boolean EndOfCode = false;
}

