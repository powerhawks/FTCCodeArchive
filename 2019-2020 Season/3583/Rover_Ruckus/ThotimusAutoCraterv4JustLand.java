package org.firstinspires.ftc.teamcode.Rover_Ruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "ThottyGoCrater: Land and Stop")
public class ThotimusAutoCraterv4JustLand extends ThotimusSkeltonAutov10 {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        useEncoders();
        boolean autoMoveDone = false;

        try {
            while (!autoMoveDone) {
                try {
                    land();
                    goForward(1, TICKSPERINCH * 15);
                    lowerLift();
                    while (liftMechanism.isBusy()){
                        Thread.sleep(100);
                    }
                    stopRobot();
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
