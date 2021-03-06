package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Auto Red 1")

////////////////////////////////////////////////////////////Put SkeletonOp into a base class and extend from LinearOpMode////////////////

public class AutoRed1 extends BaseOpMode {
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

                turnRightDeg(45);
                backwardInch(4);
                turnRightDeg(20);
                lowerGlyph();

                openGlyphArm();
                Thread.sleep(500);
                backwardInch(2);
                Thread.sleep(250);
                forwardInch(10);
                Thread.sleep(500);
                switch (vuMark) {
                    case CENTER:
                        wiggleRight();
                        break;
                    case LEFT:
                        wiggleRight();
                        break;
                    case RIGHT:
                        wiggleLeft();
                        break;
                    case UNKNOWN:
                        wiggle();
                        break;
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
        forwardInch(39); //37);//45);
        Thread.sleep(500);
    }
    void goToCenterColumn() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(34);//32);//);40);
        Thread.sleep(500);
    }
    void goToRightColumn() throws InterruptedException {
        Thread.sleep(500);
        forwardInch(30); //28);//35);
        Thread.sleep(500);
    }

    void goToPile() throws InterruptedException{
        backwardInch(2);
        turnRightDeg(180);
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


