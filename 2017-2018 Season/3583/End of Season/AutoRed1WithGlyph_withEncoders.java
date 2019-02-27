package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Red 1 Center Encoders")

////////////////////////////////////////////////////////////Put SkeletonOp into a base class and extend from LinearOpMode////////////////

public class AutoRed1WithGlyph_withEncoders extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        this.relicTrackables.activate();
        RelicRecoveryVuMark vuMark = determineVumarkPosition();
        int maxTries = 5;
        try {
            while (opModeIsActive()) {
                telemetry.addData("vuMark", vuMark.name());
                logMotors();
                closeGlyphArm();
                Thread.sleep(500);
                gLift.setPower(1);
                Thread.sleep(750);
                gLift.setPower(0);
                lowerJewel();
                Thread.sleep(1000);
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
                        if (maxTries == 0) {
                            raiseJewel();
                            done = true;
                        } else {
                            maxTries--;

                            telemetry.addData("jewelColor", "unknown");
                            idle();
                            updateColorSensor();
                        }
                    }
                    telemetry.update();
                }

                isCenter();

                turnRightDeg(90);
                Thread.sleep(500);
                gLift.setPower(-1);
                Thread.sleep(750);
                gLift.setPower(0);

                openGlyphArm();
                Thread.sleep(500);
                backwardInch(2);
                Thread.sleep(250);
                forwardInch(10);
                Thread.sleep(500);
                wiggle();
                backwardInch(2);
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
    void isCenter() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(33);
        Thread.sleep(500);
    }
    void isLeft() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(31);
        Thread.sleep(500);
    }
    void isRight() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(38);
        Thread.sleep(500);
    }
}


