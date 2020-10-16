package org.firstinspires.ftc.teamcode.Rover_Ruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "ThottyGoCraterv3")
public class ThotimusAutoCraterv3 extends ThotimusSkeltonAutov10 {
    @Override
    public void runOpMode() {
        initRobot();
        useEncoders();
        waitForStart();

        boolean autoMoveDone = false;

        try {
            while (!autoMoveDone) {
                try {
                    String goldPos = detectMinerals(true);
                    land();
                    lowerLift();
                    sample(goldPos);
                    lowerLift();
                    moveToDepotCrater();
                    lowerLift();
                    dropMarker();
                    lowerLift();
                    moveToCraterCrater();
                    lowerLift();
                    stopRobot();
                } catch (Exception e) {
                    telemetry.addData("Error", e.getMessage());
                    stopRobot();
                } finally {
                    autoMoveDone = true;
                    stopRobot();
                }
            }
            stopRobot();
        } finally {
            stopRobot();
        }
    }
}
