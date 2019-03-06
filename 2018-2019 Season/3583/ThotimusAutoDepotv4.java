package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThottyGoDepotv4")
public class ThotimusAutoDepotv4 extends ThotimusSkeltonAutov10 {
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
                    if(goldPos == "Left"){
                        sampleLeft();
                    }else if(goldPos == "Right"){
                        sampleRight();
                    }else{
                        sampleCenter();
                    }
                    lowerLift();
                    dropMarkerDepot();
                    lowerLift();
                    moveToCraterDepot();
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
