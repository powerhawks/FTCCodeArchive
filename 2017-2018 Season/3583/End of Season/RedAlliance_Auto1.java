package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Power Hawks Robotics on 12/2/2017.
 */

@Autonomous(name="RED1 Jewel & Glyph")
public class RedAlliance_Auto1 extends BaseOpMode {
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
                raiseGlyph();
                lowerJewel();
                idle();
                updateColorSensor();
                while (!done) {
                    telemetry.addData("hue", hsvValues[0]);
                    if (isRed()) {
                        telemetry.addData("jewelColor", "red");
                        knockLeft();
                        waitForMotors(2000);
                        raiseJewel();
                        Thread.sleep(1000);
                        knockRight();
                        done = true;
                    } else if (isBlue()) {
                        telemetry.addData("jewelColor", "blue");
                        knockRight();
                        waitForMotors(2000);
                        raiseJewel();
                        Thread.sleep(1000);
                        knockLeft();
                        done = true;
                    } else {
                        telemetry.addData("jewelColor", "unknown");
                        idle();
                        updateColorSensor();
                    }
                    telemetry.update();
                    Thread.sleep(1000);
                }

                // now let's drive to the glyph box
                forwardPos(3000);  // inches
                Thread.sleep(500);
                turnRightPos(75); // ticks
                lowerGlyph();
                Thread.sleep(500);
                openGlyphArm();
                forwardPos(37);
                break;
            }
            gLift.setPower(-1);
            Thread.sleep(1000);
            gLift.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        } finally {
            StopMotors();
        }
        // jewel
//            if(running){
//                lowerJewel();
//                while(!done) {
//                    updateColorSensor();
//                    if(isRed() || isBlue()) {
//                        done = true;
//                    }
//                }
//                telemetry.addLine(isBlue() ? " ball is blue" : "ball is red");
//                telemetry.update();
//                if(isRed()) { forwardInch(2);}
//                if(isBlue()) { backwardsInch(2);}
//                raiseJewel();
//                running = false;
//            }
        // movement
//                forwardInch(6);
//                stopMotors();
//                sleep(500);
//                backwardsInch(12);
//                stopMotors();
//                sleep(500);
//                TurnRightDeg(90);
//                stopMotors();
//                sleep(500);
//                TurnLeftDeg(180);

        //vuforia
//            RelicRecoveryVuMark vuMark = determineVumarkPosition();
//            telemetry.addLine("VuMark: " + vuMark.toString());
//            telemetry.update();


        idle();
    }

//    }
        long prevTime = System.currentTimeMillis();
        boolean EndOfCode = false;
    }


