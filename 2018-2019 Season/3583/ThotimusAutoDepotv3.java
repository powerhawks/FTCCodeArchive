package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThottyGoDepotv3")
public class ThotimusAutoDepotv3 extends ThotimusSkeltonAutov10 {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        useEncoders();
        boolean autoMoveDone = false;

        try {
            while (!autoMoveDone) {
                try {
                    String goldPos = detectMinerals(true);
                    land();
                    lowerLift();
                    sample(goldPos);
                    lowerLift();
                    moveToDepotDepot();
                    lowerLift();
                    dropMarker();
                    lowerLift();
                    moveToCraterDepot();
                    lowerLift();
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
