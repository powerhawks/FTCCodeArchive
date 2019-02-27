package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */
@Disabled
@Autonomous(name="Red 1 Center Glyph")

////////////////////////////////////////////////////////////Put SkeletonOp into a base class and extend from LinearOpMode////////////////

public class AutoRed1WithGlyph extends BaseOpMode {
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
//                        turnLeftTime(250);
                        Thread.sleep(1000);
                        raiseJewel();
                        Thread.sleep(500);
//                        turnRightTime(250);
                        knockRight();
                        Thread.sleep(1000);
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        knockRight();
//                        turnRightTime(250);
                        Thread.sleep(1000);
                        raiseJewel();
                        Thread.sleep(500);
//                        turnLeftTime(250);
                        knockLeft();
                        Thread.sleep(1000);
                        done = true;
                    } else {
                        telemetry.addData("jewelColor", "unknown");
                        idle();
                        updateColorSensor();
                    }
                    telemetry.update();
                    Thread.sleep(1000);
                }

                Thread.sleep(1000);
                forwardTime(700);
                Thread.sleep(500);
                turnRightTime(1600);
                Thread.sleep(500);
                gLift.setPower(-1);
                Thread.sleep(500);
                gLift.setPower(0);

                openGlyphArm();
                Thread.sleep(500);
                forwardTime(500);
                Thread.sleep(500);
                backwardTime(50);
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


