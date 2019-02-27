package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="Auto Blue 2")

public class AutoBlue2 extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // motors
        initRobot();
        waitForStart();
        boolean done = false;
        boolean running = true;
        //this.relicTrackables.activate();
        determineVumarkPosition();
        int maxTries = 5;
        try {
            while (opModeIsActive()) {
                logData();
                closeGlyphArm();
                raiseGlyph();
                lowerJewel();
                Thread.sleep(1500);
                idle();
                updateColorSensor();
                while (!done) {
                    telemetry.addData("hue", hsvValues[0]);
                    if (isRed()) {
                        telemetry.addData("jewelColor", "red");
                        knockLeft();
                        Thread.sleep(500);
                        raiseJewel();
                        Thread.sleep(500);
                        knockRight();
                        Thread.sleep(500);
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        knockRight();
                        Thread.sleep(500);
                        raiseJewel();
                        Thread.sleep(500);
                        knockLeft();
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
                backwardInch(10);
                Thread.sleep(500);

                backwardPivotRight(2100);

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

                lowerGlyph();
                openGlyphArm();
                backwardInch(2);
                Thread.sleep(500);
                forwardInch(12);
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
            } finally{
                StopMotors();
            }
        }

        void goToLeftColumn() throws InterruptedException {
            turnLeftDeg(22);
        }

        void goToCenterColumn() throws InterruptedException {
            //backwardPivotRight(2000);
            //turnRightDeg(180)
        }
        void goToRightColumn() throws InterruptedException {
            turnRightDeg(22);
            //backwardPivotRight(2200);
        }

    void goToPile() throws InterruptedException{
        backwardInch(2);
        turnRightDeg(150);
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