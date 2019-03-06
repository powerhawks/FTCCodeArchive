package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by TUCK ACT 4!!!!!!!
 */

@TeleOp(name="Servo Test")
public class ServoTestAndCallibrate extends LinearOpMode {
    //CRServo collectionWheel;
    Servo Parker;
    Servo markerGate;

    @Override
    public void runOpMode() {
        //collectionWheel = hardwareMap.crservo.get("collectionW");
        Parker = hardwareMap.servo.get("park");
        markerGate = hardwareMap.servo.get("markerGate");

        //collectionR.setDirection(CRServo.Direction.REVERSE);
        double Pos1 = .2;
        //double Min1 = .1;
        //double Max1 = .5;
        double Inc1 = .05;

        double Pos2 = .2;
        //double Min2 = .1;
        //double Max2 = .5;
        double Inc2 = .05;


        waitForStart();
        while(opModeIsActive()) {

            //Rotate.setPosition(Pos1);
            //Sampler.setPosition(Pos2);

            //rotate collector up
            //+left stick







                //Sampler positions should be between .00 and .05
        /*
            if (gamepad2.right_stick_y > 0){

                Sampler.setPosition(.14);//dropping
            }

            // -left stick
            if (gamepad2.right_stick_y < 0){

                //Change this
                Sampler.setPosition(.16);//collecting
            }

            if (gamepad2.right_stick_button){
                    Sampler.setPosition(.2);//resting
            }
*/
//  (.225 HIT) (.218 Scan) (.15 Up)

            if (gamepad2.a){
                //Pos2 = .04;
                markerGate.setPosition(1);
               // Wheel.setPower(0.75);
            } else if (gamepad2.b){
                //Pos2 = .05;
                markerGate.setPosition(0);
               // Wheel.setPower(-0.75);
            }
            if (gamepad2.x){
                //Pos2 = .06;
                //Parker.setPosition(.72);

            }
            if (gamepad2.y){
                //Pos2 = .07;
                //Parker.setPosition(.7);

            }
            //open: .5

            //Resting = .2
            //Dropping = .14
            //collecting = .16

            /*
            if(gamepad2.a){
                //collectionWheel.setPower(.5);
                //collectionR.setPower(1);
            }

            if (gamepad2.b){
                //collectionR.setPower(-1);
                //collectionWheel.setPower(-.5);

            }
            if (!gamepad2.a && !gamepad2.b){
                //collectionWheel.setPower(0);
            }
            */

            //telemetry.addData("Rotate Position: ", Rotate.getPosition());
            //telemetry.addData("Rotate Code Position: ", Pos1);

            //telemetry.addData("Rotate Position: ", Rotate.getPosition());
            //telemetry.addData("Sampler Code Position: ", Pos2);
            telemetry.addData("Peter_Parker",Parker.getPosition());


            updateTelemetry(telemetry);
        }
    }
}
