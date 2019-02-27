package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.android.internal.util.Predicate;

/* Joe Lewis June 6th 2018
   Modified Worlds Teleop to be controlled with a single controler
   Used at Boys and Girls Club
 */

@TeleOp(name="B&Gs Teleop", group="Teleop")
public class Boys_Girls extends SkeletonOpMode {
    public static final double LEFT_CLAMP_MAX = 0.55;
    public static final double LEFT_CLAMP_MIN = 0.31;
    public static final double RIGHT_CLAMP_MAX = 0.40;
    public static final double RIGHT_CLAMP_MIN = .60;
    public static final double SERVO_INCREMENT = .02;
    public static final int CYCLE_MS = 100; // ms

    long lastTime = System.currentTimeMillis() - CYCLE_MS;
    double motorSpeed = 1.0; //Multiplier for motor speed
    byte toggleWait = 5;

    @Override
    public void runOpMode() {
        try {
            initRobot();

            waitForStart();

            while (opModeIsActive()) {

                //   motors

                //Assigns the left drives to the left joystick
                lBackDrive.setPower(gamepad1.left_stick_y * motorSpeed);

                //        //Assigns the right drives to the right joystick
                rBackDrive.setPower(gamepad1.right_stick_y * motorSpeed * -1);

                //Assigns the Glyph Lift motor to the triggers
                //        gLift.setPower(gamepad2.right_trigger * 1.0);
                //        gLift.setPower(gamepad2.left_trigger * -1.0);

                // Assigns the Relic Arm to bumpers

                //relicArm.setPower(gamepad1.left_trigger * -1.0);
                //relicArm.setPower(gamepad1.right_trigger * 1.0);

                //Lift control for controller 1
                if(gamepad1.left_bumper) {
                    gLift.setPower(1);
                } else {
                    gLift.setPower(0);
                }
lkmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
                if(gamepad1.right_bumper) {
                    gLift.setPower(-1);
                } else {
                    gLift.setPower(0);
                }


                //gLift.setPower(gamepad2.dpad_up ? 1.0 : 0);
                //gLift.setPower(gamepad2.dpad_down ? -1.0 : 0);
                //gLift.setPower(gamepad2.right_trigger * -1.0);
                //   servos

                // used only for calibration
                if (gamepad1.y) {
                    jewelArm.setPosition(0.50);
                }
//
//                if (gamepad1.b) {
//                    jewelArm.setPosition(jewelArm.getPosition() - 0.01);
//                }
                // close clamp
                if (gamepad1.a) {
                    lGClamp.setPosition(LEFT_CLAMP_MAX);
                    rGClamp.setPosition(RIGHT_CLAMP_MAX);
                }
                // open clamp
                if (gamepad1.b) {
                    lGClamp.setPosition(LEFT_CLAMP_MIN);
                    rGClamp.setPosition(RIGHT_CLAMP_MIN);
                }

                if (System.currentTimeMillis() - lastTime >= CYCLE_MS) {
                    lastTime = System.currentTimeMillis();
                    // close
//                    if (gamepad2.x) {
//                        lGClamp.setPosition(lGClamp.getPosition() + 0.02);
//                        rGClamp.setPosition(rGClamp.getPosition() - 0.02);
//                    }
                    // open
                    if (gamepad2.y) {
                        lGClamp.setPosition(lGClamp.getPosition() - 0.02);
                        rGClamp.setPosition(rGClamp.getPosition() + 0.02);
                    }
                }

                //Use the gamepad 1 y button to toggle between full speed and half speed
                if(gamepad1.left_bumper){
                    motorSpeed = .5;
                    telemetry.addData("Motor Spped", motorSpeed);
                }
                else{
                    motorSpeed = 1;
                    telemetry.addData("Motor Spped", motorSpeed);
                }
//                if(gamepad1.y && toggleWait < 1) {
//                    if(motorSpeed == 1.0) {
//                        motorSpeed = 0.5;
//                        toggleWait = 5;
//                        telemetry.addData("Motor Spped", motorSpeed);
//                    } else {
//                        motorSpeed = 1;
//                        toggleWait = 5;
//                        telemetry.addData("Motor Spped", motorSpeed);
//                    }
//                }
//                toggleWait--;



                telemetry.addData("jewelArmPos", jewelArm.getPosition());
                telemetry.addData("lGClampPos", lGClamp.getPosition());
                telemetry.addData("rGClampPos", rGClamp.getPosition());
                telemetry.addData("rBackDrive", rBackDrive.getCurrentPosition());
                telemetry.addData("lBackDrive", lBackDrive.getCurrentPosition());
                telemetry.addData("lift", gLift.getCurrentPosition());
                telemetry.addData("gpad2.left_stick_y", gamepad2.left_stick_y);
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