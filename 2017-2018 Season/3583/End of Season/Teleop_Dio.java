package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.android.internal.util.Predicate;

@TeleOp(name="Dio Brando", group="Teleop")
public class Teleop_Dio extends SkeletonOpMode {
    public static final double LEFT_CLAMP_MAX = .6;
    public static final double LEFT_CLAMP_MIN = .2;
    public static final double RIGHT_CLAMP_MAX = .05;
    public static final double RIGHT_CLAMP_MIN = .42;
    public static final double SERVO_INCREMENT = .02;
    public static final int CYCLE_MS = 100; // ms

    long lastTime = System.currentTimeMillis() - CYCLE_MS;

    double leftClampPos = LEFT_CLAMP_MIN;
    double rightClampPos = RIGHT_CLAMP_MAX;

    @Override
    public void runOpMode() {
        try {
            initRobot();

            waitForStart();

            while (opModeIsActive()) {

                //   motors

                //Assigns the left drives to the left joystick
                lFrontDrive.setPower(gamepad1.left_stick_y * 1.0);
                lBackDrive.setPower(gamepad1.left_stick_y * 1.0);

                //        //Assigns the right drives to the right joystick
                rFrontDrive.setPower(gamepad1.right_stick_y * -1.0);
                rBackDrive.setPower(gamepad1.right_stick_y * -1.0);

                //Assigns the Glyph Lift motor to the triggers
                //        gLift.setPower(gamepad2.right_trigger * 1.0);
                //        gLift.setPower(gamepad2.left_trigger * -1.0);

                // Assigns the Relic Arm to bumpers

                //relicArm.setPower(gamepad1.left_trigger * -1.0);
                //relicArm.setPower(gamepad1.right_trigger * 1.0);

                gLift.setPower(gamepad2.left_trigger * -1.0);
                gLift.setPower(gamepad2.right_trigger * 1.0);

                //   servos
                //Set the relic system to desired angle above field barrier.
                if (gamepad2.dpad_down) {
                    relicJoint.setTargetPosition(0);
                    relicJoint.setPower(1);
                }
                if (gamepad2.dpad_up) {
                    relicJoint.setTargetPosition(500);
                    relicJoint.setPower(1);
                }

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

                // close clamp
                if (gamepad2.a) {
                    lGClamp.setPosition(0.55);
                    rGClamp.setPosition(0.40);
                }
                // open clamp
                if (gamepad2.b) {
                    lGClamp.setPosition(0.31);
                    rGClamp.setPosition(.60);
                }
//                if (gamepad2.dpad_left) {
//                    relicClamp.setPosition(0);
//                } else {relicClamp.setPosition(1);}
//                if (gamepad2.dpad_right) {
//                    relicPivot.setPosition(0);
//                } else {relicPivot.setPosition(1);}

                if (System.currentTimeMillis() - lastTime >= CYCLE_MS) {
                    lastTime = System.currentTimeMillis();
                    // close
//                    if (gamepad2.x) {
//                        lGClamp.setPosition(lGClamp.getPosition() + 0.02);
//                        rGClamp.setPosition(rGClamp.getPosition() - 0.02);
//                    }
                    // open
                    if (gamepad2.y) {
                        lGClamp.setPosition(lGClamp.getPosition() - 0.01);
                        rGClamp.setPosition(rGClamp.getPosition() + 0.01);
                    }
                }

                //        if (gamepad2.dpad_left) {
                //            jewelArm.setPosition(.6);
                //        }
                //        if (gamepad2.dpad_right) {
                //            jewelArm.setPosition(.3);
                //        }
                //        }

                telemetry.addData("lGClampPos", lGClamp.getPosition());
                telemetry.addData("rGClampPos", rGClamp.getPosition());
                telemetry.addData("lFrontPos", lFrontDrive.getCurrentPosition());
                telemetry.addData("rFrontPos",rFrontDrive.getCurrentPosition());
                telemetry.update();
                idle();
            }
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        } finally {
            stopMotors();
        }
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