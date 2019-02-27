package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red2 Autonomous")
public class JAWLAutonomous2Red3796 extends JAWLAutonomous3796 {

    @Override
    public void runOpMode() {
        super.side = Side3796.Red2;
        try {
            super.runOpMode();
        } catch (InterruptedException e) {
            stop();
        }
    }
}