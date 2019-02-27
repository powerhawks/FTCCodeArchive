package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * January 7, 2018
 * Created by Chase Galey and Joe Lewis
 * <p>
 * Autonomous navigation helper class
 * This is a very long and complicated file
 * Good luck reading it
 */

public class OldAutonomousNavigation {
    public static boolean isRunning = false;
    private static boolean isDone = false;

    //This method has all the logic for navigating to the cryptoboxes for both platforms and both alliances
    //At the start of this method we assume one of the jewels has been knocked off and the robot is still balanced on the platform
    //In this method we only handle moving the robot to the glyph box, we do not open the grabbers here
    //For side definitions see Side3796.java
    public static void navigateToCryptobox(Side3796 side, RelicRecoveryVuMark vumark, JAWLDrive3796 drive, Telemetry tele) {
        int[] oldPositions;
        int ticks = 0;
        int[] targetPositions;
        int variance = 50;
        int sleepTime = 50;
        double globalPower = 0.75;

        //Fallback method if no Vumark was found then drive to cryptobox and place glyph in center column
        if (vumark == RelicRecoveryVuMark.UNKNOWN) {
            vumark = RelicRecoveryVuMark.CENTER;
        }


        //First we check what platform we are on

        if (side == Side3796.Red1) {
            //Now in each if statement we check what column the vumark has told us to place the glyph into
            //We know at this point that a vumark was found because of the fallback method above

            if (vumark == RelicRecoveryVuMark.RIGHT) {
                //If we were told to go to the right column we need to backup towards the glyphbox, rotate 90 degrees then drive towards the glyph box

                //First step: backup 32.185 inches to the center of the right column
                //(120−84) − (((7.63×3)/2)−7.63) = 32.185

                //We take note of the old positions before we send the backwards commmand
                //We really should have a handler for this
                oldPositions = drive.getPositions();
                ticks = inchesToTicks(32.185);
                targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
                drive.backwards(ticks, globalPower);

                //See our drive class for how this works
                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }

                //Now we rotate 90 degrees
                /*
                 * Math for going from 90 degrees to a number of ticks
                 * diameter of wheels = 9.5cm
                 * width of robot = ‪36.83‬ cm
                 * we assume the wheels are at the edge of the robot
                 * number of ticks per revolution of motor shaft on andymark 40 = 1120
                 *
                 * 9.5 * pi = 29.85
                 * each revolution of our wheel turns the robot 29.85 centimeters (if both wheels are moving in opposing directions)
                 *


               .  * (‪36.83‬ * pi) = 115.705 cm
                 * to complete a 360 degree turn we need to move 115.705cm
                 * 115.705/4 = 28.92621cm
                 * to complete a 90 degree turn we need to move 28.92621cm
                 *
                 * 28.92621cm/29.85cm = 0.96921052631578947368421052631579
                 * We need to complete 0.96921052631578947368421052631579 revolutions of each wheel to turn 90 degrees
                 *
                 * 0.96921052631578947368421052631579 * 1120 = 1086
                 * we need to move both motors 1086 ticks to complete a 90 degree turn
                 */


                oldPositions = drive.getPositions();
                ticks = 1497;
                targetPositions = new int[]{oldPositions[0], oldPositions[1] + ticks};
                //These are swapped because our drive methods are swapped when using turn left and turn right
                drive.turnLeft(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], 10)) {
                    sleep(sleepTime);
                }

                //Now we drive forwards 15 inches to the cryptobox
                oldPositions = drive.getPositions();
                ticks = inchesToTicks(15);
                targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
                drive.forward(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }


            } else if (vumark == RelicRecoveryVuMark.CENTER) {

                //If we were told to go to the center column we need to backup towards the glyphbox, rotate 90 degrees then drive towards the glyph box

                //First step: backup 39.815 inches to the center of the right column

                oldPositions = drive.getPositions();
                ticks = inchesToTicks(39.815);
                targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
                drive.backwards(ticks, globalPower);

                //See our drive class for how this works
                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }

                sleep(sleepTime + sleepTime);

                //Now we rotate 90 degrees
                oldPositions = drive.getPositions();
                ticks = 1497;
                targetPositions = new int[]{oldPositions[0], oldPositions[1] + ticks};
                //These are swapped because our drive methods are swapped when using turn left and turn right
                drive.turnLeft(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], 10)) {
                    sleep(sleepTime);
                }

                //Now we drive forwards 15 inches to the cryptobox
                oldPositions = drive.getPositions();
                ticks = inchesToTicks(15);
                targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
                drive.forward(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }

            } else if (vumark == RelicRecoveryVuMark.LEFT) {
                //If we were told to go to the center column we need to backup towards the glyphbox, rotate 90 degrees then drive towards the glyph box

                //First step: backup 47.445 inches to the center of the right column

                oldPositions = drive.getPositions();
                ticks = inchesToTicks(47.445);
                targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
                drive.backwards(ticks, globalPower);

                //See our drive class for how this works
                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }

                //Now we rotate 90 degrees
                oldPositions = drive.getPositions();
                ticks = 1497;
                targetPositions = new int[]{oldPositions[0], oldPositions[1] + ticks};
                //These are swapped because our drive methods are swapped when using turn left and turn right
                drive.turnLeft(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], 10)) {
                    sleep(sleepTime);
                }

                //Now we drive forwards 15 inches to the cryptobox
                oldPositions = drive.getPositions();
                ticks = inchesToTicks(15);
                targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
                drive.forward(ticks, globalPower);

                while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                    sleep(sleepTime);
                }
            }


        } else if (side == Side3796.Red2) {


            //First we backup off the platform 24 inches
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(24);
            targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
            drive.backwards(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Then we rotate 90 degres clockwise
            oldPositions = drive.getPositions();
            ticks = 1497;
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1]};
            drive.turnRight(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Drive forward to center our robot with the specfic column
            oldPositions = drive.getPositions();
            if (vumark == RelicRecoveryVuMark.RIGHT) {
                ticks = inchesToTicks(3.815);
            } else if (vumark == RelicRecoveryVuMark.CENTER) {
                ticks = inchesToTicks(11.445);
            } else if (vumark == RelicRecoveryVuMark.LEFT) {
                ticks = inchesToTicks(19.075);
            }
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Turn another 90 degrees to be in front of column
            oldPositions = drive.getPositions();
            ticks = 1497;
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1]};
            drive.turnRight(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Drive forwards 15 inches to insert the glyph
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(15);
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);

            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }


        } else if (side == Side3796.Blue1) {


            oldPositions = drive.getPositions();
            if (vumark == RelicRecoveryVuMark.RIGHT) {
                ticks = inchesToTicks(32.185);
            } else if (vumark == RelicRecoveryVuMark.CENTER) {
                ticks = inchesToTicks(39.815);
            } else if (vumark == RelicRecoveryVuMark.LEFT) {
                ticks = inchesToTicks(39.815);
            }

            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);

            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Now we rotate 90 degrees
            oldPositions = drive.getPositions();
            ticks = 1497;
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1]};
            drive.turnRight(ticks, globalPower);

            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Now we drive forwards 15 inches to the cryptobox
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(15);
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);

            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }


        } else if (side == Side3796.Blue2) {

            //First we backup off the platform 24 inches
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(24);
            targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
            drive.backwards(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Then we rotate 90 degres counter-clockwise
            oldPositions = drive.getPositions();
            ticks = 1497;
            targetPositions = new int[]{oldPositions[0], oldPositions[1] + ticks};
            drive.turnLeft(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Drive forward to center our robot with the specfic column
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(3.815);

            if (vumark == RelicRecoveryVuMark.RIGHT) {
                ticks = inchesToTicks(3.815);
            } else if (vumark == RelicRecoveryVuMark.CENTER) {
                ticks = inchesToTicks(11.445);
            } else if (vumark == RelicRecoveryVuMark.LEFT) {
                ticks = inchesToTicks(19.075);
            }
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Turn another 90 degrees to be in front of column
            oldPositions = drive.getPositions();
            ticks = 1497;
            targetPositions = new int[]{oldPositions[0], oldPositions[1] + ticks};
            drive.turnLeft(ticks, globalPower);
            //Wait till motors have reached their positions
            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }

            //Drive forwards 15 inches to insert the glyph
            oldPositions = drive.getPositions();
            ticks = inchesToTicks(15);
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
            drive.forward(ticks, globalPower);

            while (drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
                sleep(sleepTime);
            }
        }

        isDone = true;
    }

    public static boolean isDone() {
        return isDone;
    }

    private static int inchesToTicks(double inches) {
        /*
         * Ticks per revolution on an andymark 40 = 1120
         * Diameter of wheels = 9.5cm or 3.740157in
         *
         * 3.740157 * pi = 11.75
         * For every revolution our wheels travel 11.75 inches
         */

        double neededRevolutions = inches / 11.75;
        int ticks = (int) (neededRevolutions * 1120);
        return ticks;
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            //oops
        }
    }
}
