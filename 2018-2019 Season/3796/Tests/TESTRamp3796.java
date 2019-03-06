package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.KowallskiRamp3796;

@Autonomous(name = "Ramp Test", group = "X")
@Disabled
public class TESTRamp3796 extends LinearOpMode {

    KowallskiRamp3796 ramp = new KowallskiRamp3796(hardwareMap.dcMotor.get("rampMotor"), hardwareMap.dcMotor.get("rampExtender"));

    @Override
    public void runOpMode() throws InterruptedException {
        ramp.liftOrLower(1);
        sleep(1000);

        ramp.stopMovementRmp();
        sleep(3000);

        ramp.liftOrLower(-1);
        sleep(1000);
    }
}
