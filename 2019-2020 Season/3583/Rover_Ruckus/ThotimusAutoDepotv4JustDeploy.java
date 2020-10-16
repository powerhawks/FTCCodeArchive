package org.firstinspires.ftc.teamcode.Rover_Ruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "ThottyGoDepot: Land and Deploy")
public class ThotimusAutoDepotv4JustDeploy extends ThotimusSkeltonAutov10 {
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
                    goForward(1, (long) ((TICKSPERINCH*WIDTHOFTILE*4.5))-(TICKSPERINCH*9));
                    dropMarker();
                    goBackwards(1, TICKSPERINCH*WIDTHOFTILE*3);
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
