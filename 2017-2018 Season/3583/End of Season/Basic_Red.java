package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Red Basic", group="Autonomous")

////////////////////////////////////////////////////////////Put SkeletonOp into a base class and extend from LinearOpMode////////////////

public class Basic_Red extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        this.relicTrackables.activate();
        try {
            while (opModeIsActive()) {
                closeGlyphArm();
                Thread.sleep(500);
                gLift.setPower(1);
                Thread.sleep(500);
                gLift.setPower(0);
                lowerJewel();
                Thread.sleep(2000);
                idle();
                updateColorSensor();
                while (!done) {
                    telemetry.addData("hue", hsvValues[0]);
                    if (isRed()) {
                        telemetry.addData("jewelColor", "red");
                        knockLeft();
                        //turnLeftPos(10);
                        Thread.sleep(2000);
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        //turnRightPos(10);
                        knockRight();
                        Thread.sleep(2000);
                        done = true;
                    } else {
                        telemetry.addData("jewelColor", "unknown");
                        idle();
                        updateColorSensor();
                    }
                    telemetry.update();
                    Thread.sleep(1000);
                }
                raiseJewel();
                break;
            }
            //gLift.setPower(-1);
            //Thread.sleep(1000);
            //gLift.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        } finally {
            StopMotors();
        }
    }
}


