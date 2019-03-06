package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Jay and Edwin of the Weeb Group 12/4/18
 */

@TeleOp(name = "LifterTest")

public class Lifter_Test extends LinearOpMode{

    public DcMotor RLifter;
    public DcMotor LLifter;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {


            RLifter = hardwareMap.dcMotor.get("liftRight");
            LLifter = hardwareMap.dcMotor.get("liftLeft");

            RLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.a) {
                RobotDownInch(.125);
                sleep(200);
            }
            if (gamepad2.b) {
                RobotDownInch(.25);
                sleep(200);
            }
            if (gamepad2.x){
                RobotDownInch(.75);
                sleep(200);
            }
            if (gamepad2.y){
                RobotDownInch(7); //6.875
                sleep(200);
            }

            if (gamepad2.dpad_up) {
                RLifter.setPower(1);
                LLifter.setPower(1);
            }
            //motor.setPower(gamepad2.)
            //Hook down
            if (gamepad2.dpad_down) {
                RLifter.setPower(-1);
                LLifter.setPower(-1);
            }
            if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                RLifter.setPower(0);
                LLifter.setPower(0);
            }
        }
    }
        //Lower
        public void RobotDownInch ( double inch){
                double TPI = ((6.5 * 1680) / 3);//gear ratio times ticks per revolution

                RobotDownTicks((int) (TPI * inch));
            }
            public void RobotDownTicks ( int ticks){
                int targetH = RLifter.getCurrentPosition() - ticks;
                RLifter.setTargetPosition(targetH);
                try {
                    boolean done = false;
                    while (!done) {
                        LLifter.setPower(-1.0);
                        //logMotors();
                        if (!RLifter.isBusy()) {
                            done = true;
                        }
                        idle();
                    }
                } finally {
                    RLifter.setPower(0.0);
                    LLifter.setPower(0.0);
                }
        }


//////////////////////////////READ//////////////////////////////////
    /* 1. Do the test for the collection arm.
        2. To the Autonomous
        3. Do the Teleop
        4. Other
        By.Jay Bridgman 12/4/18
     */



}
