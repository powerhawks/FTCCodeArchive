package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by Tyler Lang of Clan Farquad on 11/10/2018.
 */

@TeleOp(name="Collection Test")
public class CollectionTest extends LinearOpMode {
    CRServo collectionWheel;
    CRServo collectionR;

    @Override
        public void runOpMode() {
        collectionWheel = hardwareMap.crservo.get("collectionL");
        //collectionR = hardwareMap.crservo.get("collectionR");
        //collectionR.setDirection(CRServo.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()) {
            if(gamepad2.a){
                collectionWheel.setPower(.5);
                //collectionR.setPower(1);
            }

            if (gamepad2.b){
                //collectionR.setPower(-1);
                collectionWheel.setPower(-.5);

            }
            if (!gamepad2.a && !gamepad2.b){
                collectionWheel.setPower(0);
            }
        }
    }
}
