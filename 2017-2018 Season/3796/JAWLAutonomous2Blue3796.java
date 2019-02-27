package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue2 Autonomous")
public class JAWLAutonomous2Blue3796 extends JAWLAutonomous3796 {

    @Override
    public void runOpMode() {
        super.side = Side3796.Blue2;
        try {
            super.runOpMode();
        } catch (InterruptedException e) {
            stop();
        }
    }
}