package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 1/2/2018.
 */
@Autonomous(name="VuMark Test", group="TestAuto")

public class VuMarkTest extends BaseOpMode{
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
                determineVumarkPosition();
                telemetry.addData("vuMark", vuMark.toString());
                telemetry.update();
                idle();
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

