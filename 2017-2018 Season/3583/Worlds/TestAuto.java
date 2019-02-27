package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 1/2/2018.
 */
@Autonomous(name="Auto Test", group="TestAuto")

public class TestAuto extends BaseOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        this.relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        try {
            while (opModeIsActive()) {


                backwardPivotLeft(2000);

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

