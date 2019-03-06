package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MiscAutoTest")
public class MiscAutoTest extends ThotimusSkeltonAutov10 {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        useEncoders();
        boolean autoMoveDone = false;

        try {
            while (!autoMoveDone) {
                try {
                    gyroDrive(1, 10, 0);
                } catch (Exception e) {
                    telemetry.addData("Error", e.getMessage());
                } finally {
                    autoMoveDone = true;
                }
            }
            stopRobot();
        } finally {
            stopRobot();
        }
    }
}
