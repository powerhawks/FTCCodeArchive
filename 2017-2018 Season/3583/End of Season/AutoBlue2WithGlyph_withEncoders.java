package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Blue 2 Center Encoders")

////////////////////////////////////////////////////////////Put SkeletonOp into a base class and extend from LinearOpMode////////////////

public class AutoBlue2WithGlyph_withEncoders extends BaseOpMode {
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
                raiseGlyph();
                lowerJewel();
                Thread.sleep(1000);
                idle();
                updateColorSensor();
                while (!done) {
                    telemetry.addData("hue", hsvValues[0]);
                    if (isRed()) {
                        telemetry.addData("jewelColor", "red");
                        knockRight();
                        Thread.sleep(500);
                        raiseJewel();
                        Thread.sleep(500);
                        knockLeft();
                        Thread.sleep(500);
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        knockLeft();
                        Thread.sleep(500);
                        raiseJewel();
                        Thread.sleep(500);
                        knockRight();
                        Thread.sleep(500);
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

                Thread.sleep(500);
                backwardInch(27);
                Thread.sleep(500);

                turnLeftDeg(105);

                isCenter();

                turnLeftDeg(105);
                Thread.sleep(500);
                lowerGlyph();
                openGlyphArm();
                Thread.sleep(500);
                backwardInch(2);
                Thread.sleep(500);
                forwardInch(7);
                Thread.sleep(500);
                wiggle();
                backwardInch(3);
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
        forwardInch(7);
        Thread.sleep(500);
    }
    void isLeft() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(3.5);
        Thread.sleep(500);
    }
    void isRight() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(9.5);
        Thread.sleep(500);
    }
}


