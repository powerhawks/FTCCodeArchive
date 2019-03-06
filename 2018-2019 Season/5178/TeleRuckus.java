package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.android.internal.util.Predicate;

@TeleOp(name="TeleRuckus", group="Teleop")
public class TeleRuckus extends SkeletonOpMode {

    double CollectorPos = .0;

    double RotatePos = 1;
    //Boolean ArmPosLock = false;
    /*
    public static final double LEFT_CLAMP_MAX = .6;
    public static final double LEFT_CLAMP_MIN = .2;
    public static final double RIGHT_CLAMP_MAX = .05;
    public static final double RIGHT_CLAMP_MIN = .42;
    public static final double SERVO_INCREMENT = .02;
    public static final int CYCLE_MS = 100; // ms

    long lastTime = System.currentTimeMillis() - CYCLE_MS;

    double leftClampPos = LEFT_CLAMP_MIN;
    double rightClampPos = RIGHT_CLAMP_MAX;
    */


    @Override
    public void runOpMode() {

        initRobot();

        //collectionRotate.setPosition(1);//Resting
        waitForStart();
       // collectionRotate.setPosition(1);//Resting

        while (opModeIsActive()) {
            // Motors

            //Tank Drive
            //LDrive.setPower(gamepad1.left_stick_y * -1.0);
            //RDrive.setPower(gamepad1.right_stick_y * 1.0);


            telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_x);telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_x);


            //Arcade Drive
            if ((gamepad1.right_stick_x > -.01 && gamepad1.right_stick_x < .01) && (gamepad1.left_stick_y > -.01 && gamepad1.left_stick_y < .01)) {
                //at rest
                LDrive.setPower(0);
                RDrive.setPower(0);
            }
            if (gamepad1.right_stick_x > -.01 && gamepad1.right_stick_x < .01) {
                //forward and backwards

                //reversed polarity
                LDrive.setPower(gamepad1.left_stick_y * 1.0);
                RDrive.setPower(gamepad1.left_stick_y * -1.0);
            }
            if (gamepad1.left_stick_y > -.01 && gamepad1.left_stick_y < .01) {
                //turning
                LDrive.setPower(gamepad1.right_stick_x * -1.0);
                RDrive.setPower(gamepad1.right_stick_x * -1.0);
            } else {
                if ((gamepad1.left_stick_y > 0) && (gamepad1.right_stick_x > -.01 && gamepad1.right_stick_x < .01)) {

                }
                //forward and turning
                //reversed polarity on forward/backward

                if (gamepad1.left_stick_y < .01) {
                    // forward
                    if (gamepad1.right_stick_x > .01) {
                        //right turn
                        LDrive.setPower((gamepad1.left_stick_y * 1.0));
                        RDrive.setPower((gamepad1.left_stick_y * -1.0) - gamepad1.right_stick_x);

                    }

                    if (gamepad1.right_stick_x < -.01) {
                        //left turn
                        LDrive.setPower(gamepad1.left_stick_y * 1.0 - gamepad1.right_stick_x);
                        RDrive.setPower(gamepad1.left_stick_y * -1.0);
                    }

                }
                if (gamepad1.left_stick_y > -.01) {
                    //backward
                    if (gamepad1.right_stick_x > .01) {
                        //right turn
                        telemetry.addData("Here", 1);
                        LDrive.setPower((gamepad1.left_stick_y * 1.0)); //-1 originally
                        RDrive.setPower(-1 *((gamepad1.left_stick_y * 1.0) - gamepad1.right_stick_x)); //originally * 1

                    }
                    if (gamepad1.right_stick_x < -.01) {
                        //left turn
                        telemetry.addData("Here", 2);
                        LDrive.setPower(gamepad1.left_stick_y * 1.0 + gamepad1.right_stick_x);
                        RDrive.setPower(gamepad1.left_stick_y * -1.0);
                    }

                }
                telemetry.addData("dummy", 1);
                updateTelemetry(telemetry);

                //Tank Drive
                //LDrive.setPower((gamepad1.right_stick_x * -0.5) + (gamepad1.left_stick_y * -0.5));
                //RDrive.setPower((gamepad1.right_stick_x * -0.5) + (gamepad1.left_stick_y * 0.5));
            }
            if (gamepad2.a){
                Parker.setPosition(.82);
            }
            if(gamepad2.b){
                Parker.setPosition(0);
            }
            if(gamepad2.left_bumper){
                markerGate.setPosition(1);
            }
            if (gamepad2.right_bumper){
                markerGate.setPosition(0);
            }

            //liftLeft.setPower(gamepad2.left_stick_y * 1);
            //liftRight.setPower(gamepad2.left_stick_y * 1);


            //Hook up WORKS

            if (gamepad2.dpad_up) {
                liftRight.setPower(1);
                liftLeft.setPower(1);
            }
            //motor.setPower(gamepad2.)
            //Hook down WORKS
            if (gamepad2.dpad_down) {
                liftRight.setPower(-1);
                liftLeft.setPower(-1);
            }
            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                liftRight.setPower(0);
                liftLeft.setPower(0);
            }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////Sampler HERE!!!!/////////////////////////////////////////////////

            //Collector Arm WORKS
            //out/in
            //CollectorArm.setPower(gamepad2.right_stick_y);
            //Sampling
            /*

             */
            if (gamepad2.dpad_right) {
                //Sampler.setPosition(.23);
                // arm up
            }
            if (gamepad2.dpad_left) {
                //Sampler.setPosition(.17);
                //arm down
            }


            //collectionR.setPower(0);
            // Collection Wheel WORKS

/*
            if (gamepad2.a) {
                collectionWheel.setPower(-0.8);
                //intake
            } else if (gamepad2.b) {
                //outtake
                collectionWheel.setPower(0.8);

            } else if (!gamepad2.a && !gamepad2.b) {
                collectionWheel.setPower(0);
                //neutralize wheel intake
            }


            //Raise Collection Arm WORKS
            if (gamepad2.y) {
                CollectionRaise.setPower(-1);
            }
            if (gamepad2.x) {
                CollectionRaise.setPower(1);
            }
            if (!gamepad2.y && !gamepad2.x) {
                CollectionRaise.setPower(0);
            }

            // Collector Cont.


            //rotate collector up
            //+left stick


            //collectionRotate.setPower(-gamepad2.left_stick_y);


            // dropping = .2, storing = .5, collecting = .75, rest = 1;

            //Tale out sleeps, add button to each if


            ////////////////////////////////////////////////////ARM ROTATE///////////////////////////////////////////////////////////////////////////////////
            //incremental Collector Rotate

            if (gamepad2.left_stick_y > .1 && RotatePos < 1){
                RotatePos += .00001;
            }
            else if (gamepad2.left_stick_y < -.1 && RotatePos > 0){
                RotatePos -= .00001;
            }
            if (gamepad2.left_stick_button){
                RotatePos = .05;
            }
            collectionRotate.setPosition(RotatePos);

*/


            /*
            //Four Position Collector Rotate
            if (ArmPosLock == false) {
                if (gamepad2.left_stick_y > .2 && collectionRotate.getPosition() - .5 < .01) {  //.5

                    collectionRotate.setPosition(.05);//Dropping
                    ArmPosLock = true;
                    sleep(25);
                    OpenGate();

                    //collectionRotate.setPower(-.75);
                }
                //rotate collector down
                // -left stick
                else if ((gamepad2.left_stick_y > .2 && collectionRotate.getPosition() - .75 < .01) || (gamepad2.left_stick_y < -.2 && collectionRotate.getPosition() - .05 < .01)) {
                    collectionRotate.setPosition(.5);//holding
                    ArmPosLock = true;
                    CloseGate();
                    //collectionRotate.setPower(0);

                }
                else if ((gamepad2.left_stick_y > .2 && collectionRotate.getPosition() - 1 < .01) || (gamepad2.left_stick_y < -.2 && collectionRotate.getPosition() - .5 < .01)) {
                    collectionRotate.setPosition(.75);//collecting
                    ArmPosLock = true;
                    CloseGate();
                }

                else if ((gamepad2.left_stick_y < -.2 && collectionRotate.getPosition() - .75 < .01)) {
                    collectionRotate.setPosition(1);//Resting
                    ArmPosLock = true;
                    CloseGate();
                    //collectionRotate.setPower(.75);
                }
            }
            if (ArmPosLock == true) {
                if (gamepad2.left_stick_y == 0) {
                    ArmPosLock = false;
                }
            }

           */


            //telemetry code
            //telemetry.addData("Collector Rotate Value", collectionRotate.getPosition());
            /*
            telemetry.addData("CollectorPos Value", CollectorPos);
            telemetry.addData("Collector Rotate", collectionRotate.getPosition());
            //telemetry.addData("Arm Position Lock ", ArmPosLock);
            telemetry.addData("Left Stick Pos ", gamepad2.left_stick_y);
            telemetry.addData("Shift Down", gamepad2.left_stick_y < -.2);
            telemetry.addData("Shift Up", gamepad2.left_stick_y > .2 && collectionRotate.getPosition() - .75 < .01);

            updateTelemetry(telemetry);
*/
                /*
                gLift.setPower(gamepad2.left_trigger * -1.0);
                gLift.setPower(gamepad2.right_trigger * 1.0);
                */
            //   servos
            //Set the relic system to desired angle above field barrier.

                /*
                if (gamepad2.dpad_down) {
                    relicJoint.setTargetPosition(0);
                    relicJoint.setPower(1);
                }
                if (gamepad2.dpad_up) {
                    relicJoint.setTargetPosition(500);
                    relicJoint.setPower(1);
                }
                */

            //Extends and retracts relic system based on trigger input.

            //Assigns the clamp's servo's positions to buttons.
            // close left clamp

//                if (System.currentTimeMillis() - lastTime >= CYCLE_MS) {
//                    if (gamepad2.left_stick_x > 0) {
//                        if (leftClampPos < LEFT_CLAMP_MAX) {
//                            leftClampPos += SERVO_INCREMENT;
//                            lGClamp.setPosition(leftClampPos);
//                        }
//                        // open left clamp
//                    } else if (gamepad2.left_stick_x < 0) {
//                        if (leftClampPos > LEFT_CLAMP_MIN) {
//                            leftClampPos -= SERVO_INCREMENT;
//                            lGClamp.setPosition(leftClampPos);
//                        }
//                    }
//                    // close right clamp
//                    if (gamepad2.right_stick_x < 0) {
//                        if (rightClampPos > RIGHT_CLAMP_MIN) {
//                            rightClampPos -= SERVO_INCREMENT;
//                            rGClamp.setPosition(rightClampPos);
//                        }
//                        // open right clamp
//                    } else if (gamepad2.left_stick_x > 0) {
//                        if (leftClampPos > RIGHT_CLAMP_MAX) {
//                            leftClampPos += SERVO_INCREMENT;
//                            rGClamp.setPosition(rightClampPos);
//                        }
//                    }
//
//                }


                /*




                /*
                telemetry.addData("lGClampPos", lGClamp.getPosition());
                telemetry.addData("rGClampPos", rGClamp.getPosition());
                telemetry.addData("lFrontPos", lFrontDrive.getCurrentPosition());
                telemetry.addData("rFrontPos",rFrontDrive.getCurrentPosition());
                telemetry.update();
                */

        }


//    public void extendArm() {
//        while(gamepad2.right_trigger > 0) {
//            relicArm.setPosition(2000);
//            if(gamepad2.right_trigger == 0) {
//                break;
//            }
//        }
//    }
//    public void retractArm() {
//        while(gamepad2.left_trigger < 0) {
//            relicArm.setPosition(0.0);
//            if(gamepad2.left_trigger == 0) {
//                break;
//        }
//    }

    }
}