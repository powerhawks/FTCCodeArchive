package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Auto Red 2")

public class AutoRed2 extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        //relicTrackables.activate();
        determineVumarkPosition();
        int maxTries = 5;
        try {
            while (opModeIsActive()) {
                logData();
                closeGlyphArm();
                raiseGlyph();
                lowerJewel();
                Thread.sleep(2000);
                idle();
                updateColorSensor();
                while (!done) {
                    telemetry.addData("hue", hsvValues[0]);
                    if (isRed()) {
                        telemetry.addData("jewelColor", "red");
                        knockRight();
                        Thread.sleep(1000);
                        raiseJewel();
                        Thread.sleep(500);
                        knockLeft();
                        Thread.sleep(1000);
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        knockLeft();
                        Thread.sleep(1000);
                        raiseJewel();
                        Thread.sleep(500);
                        knockRight();
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

                Thread.sleep(1000);
                forwardInch(22);
                Thread.sleep(500);

                //turnLeftDeg(90);

                switch (vuMark) {
                    case CENTER:
                        goToCenterColumn();
                        break;
                    case LEFT:
                        goToLeftColumn();
                        break;
                    case RIGHT:
                        goToRightColumn();
                        break;
                    case UNKNOWN:
                        goToCenterColumn();
                        break;
                }

                //turnRightDeg(90);
                Thread.sleep(500);
                lowerGlyph();
                openGlyphArm();
                backwardInch(2);
                Thread.sleep(500);
                forwardInch(7);
                Thread.sleep(500);
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    wiggleRight();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    wiggleLeft();
                } else {
                    wiggle();
                }
                backwardInch(2);
                //goToPile();
                break;
            }

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        } finally {
            StopMotors();
        }
    }

    void goToLeftColumn() throws InterruptedException {
        Thread.sleep(500);
        turnLeftDeg(35);
        forwardInch(12);
        Thread.sleep(500);
    }
    void goToCenterColumn() throws InterruptedException {
        Thread.sleep(500);
        turnLeftDeg(25);
        forwardInch(10);
        Thread.sleep(500);
    }
    void goToRightColumn() throws InterruptedException {
        Thread.sleep(500);
        turnLeftDeg(15);
        forwardInch(10);
        Thread.sleep(500);
    }
    void goToPile() throws InterruptedException{
        backwardInch(2);
        turnLeftDeg(150);
        forwardInch(36);
        knockLeft();
        closeGlyphArm();
        knockRight();
        turnRightDeg(180);
        gLift.setPower(4);
        forwardInch(38);
        wiggle();
        openGlyphArm();
    }
}


