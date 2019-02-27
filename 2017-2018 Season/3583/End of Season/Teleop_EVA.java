package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.android.internal.util.Predicate;

@TeleOp(name="Evangelion Unit-01", group="Teleop")
@Disabled
public class Teleop_EVA extends  SkeletonOpMode{


    @Override
    public void runOpMode() {

        boolean aState = false;
        boolean xState = true;
        lDrive.setPower(gamepad1.left_stick_y * 1);
        rDrive.setPower(gamepad1.right_stick_y * -1);
        shooter.setPower(gamepad2.right_trigger * -1);
        collect.setPower(gamepad2.left_trigger * 1);

        if (gamepad1.x) {

        }

        if (gamepad2.right_bumper)
        {
            shooter.setPower(1);
        }

        if (gamepad2.left_bumper)
        {
            shooter.setPower(0);
            collect.setPower(0);
        }
        if (gamepad2.a)
        {
            shooter.setPower(-1);
        }
        else if (gamepad2.y)
        {
            shooter.setPower(1);
        }
        else
        {
            shooter.setPower(0);
        }
       if(gamepad2.b)
        {
            collect.setPower(1);
        }
        else
        {
            collect.setPower(0);
        }
    }
}
