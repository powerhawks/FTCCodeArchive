package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Power Hawks Robotics on 1/2/2018.
 */
@Disabled
@Autonomous(name="Gyro Test", group="TestAuto")

public class GyroTest extends BaseOpModeGyro {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;

        try {
            while (opModeIsActive()) {
                this.gyroDrive(.5, 12, 0 );
                this.gyroTurn(.7, 90);
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

