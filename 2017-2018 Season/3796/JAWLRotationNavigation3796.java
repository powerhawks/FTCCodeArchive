package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * November 14, 2017
 * Chase Galey and Joe Lewis
 *
 * Rotation autonomous navigation
 */

public class JAWLRotationNavigation3796 {

    //This is the power value we will use for all drive orders within this class
    private static double power = 0.6;
    //We initalize a state enum
    private static NavigationState state = NavigationState.GETOFFPLATFORM;
    //We initalize an enum we want to set after the wait is done
    private static NavigationState stateAfterWait = NavigationState.WAIT;
    private static JAWLDriveOrder3796 driveOrder = null;
    public static boolean isDone = false;
    private static int globalWaitInt = 30;
    private static int waitLimit = 600;
    private static int i = 0;

    //This resets all of our variables
    public static void resetNavigation(JAWLDrive3796 drive) {
        state = NavigationState.GETOFFPLATFORM;
        stateAfterWait = NavigationState.WAIT;
        driveOrder = null;
        isDone = false;
        globalWaitInt = 30;
        i = 0;
        drive.resetEncoders();
    }

    public static void navigateToCryptobox(Side3796 side, RelicRecoveryVuMark vumark, JAWLDrive3796 drive, Telemetry tele) {
        if(vumark == RelicRecoveryVuMark.UNKNOWN) {
            tele.addLine("No Vumark found! Defaulting to center collumn");
            vumark = RelicRecoveryVuMark.CENTER;
        }

        if(side == Side3796.Red1) {

            switch (state) {
                case WAIT:
                    if(driveOrder.isDone() || i > waitLimit) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    //First we get our robot off the platform by driving forwards 2 feet
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = inchesToTicks(30);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(36);
                    } else {
                        ticks = inchesToTicks(46);
                    }
                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.BACKWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    //Now we rotate right to an angle somewhere between 90 and 180
                    ticks = 0;
                    if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = degreesToTicks(60);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        //We turn the robot 45 degrees to align with the center column
                        ticks = degreesToTicks(60);
                    } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = degreesToTicks(60);
                    }
                    driveOrder = new JAWLDriveOrder3796(drive, degreesToTicks(90), JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARD;
                    break;
                case FORWARD:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(6), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.DONE;
                    break;
                case DONE:
                    isDone = true;
                    break;
            }




        } else if (side == Side3796.Red2) {

            switch (state) {
                case WAIT:
                    if(driveOrder.isDone() || i > waitLimit) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    //First we drive backwards 2 feet to get our robot off the platform
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(24), JAWLDriveOrder3796.OrderType.BACKWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    //Now we rotate right to an angle somewhere between 90 and 180
                    int ticks = 0;
                    if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = degreesToTicks(143);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        //We turn the robot 45 degrees to align with the center column
                        ticks = degreesToTicks(165);
                    } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                        //this is the complicated column
                        //you need to trace this code because I am not going to explain it to you
                        ticks = degreesToTicks(104);
                    }
                    if (vumark == RelicRecoveryVuMark.RIGHT) {
                        driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    } else {
                        driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    }
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        stateAfterWait = NavigationState.BACKWARD;
                    } else {
                        stateAfterWait = NavigationState.FORWARD;
                    }
                    break;
                case BACKWARD:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(16), JAWLDriveOrder3796.OrderType.BACKWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE2;
                    break;
                case ROTATE2:
                    driveOrder = new JAWLDriveOrder3796(drive, degreesToTicks(90), JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARD;
                    break;
                case FORWARD:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(10), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(6), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    }
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.DONE;
                    break;
                case DONE:
                    isDone = true;
                    break;
            }

        } else if (side == Side3796.Blue1) {

            switch (state) {
                case WAIT:
                    if(driveOrder.isDone() || i > waitLimit) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    //First we get our robot off the platform by driving forwards 2 feet
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = inchesToTicks(26);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(34);
                    } else if (vumark == RelicRecoveryVuMark.RIGHT){
                        ticks = inchesToTicks(38);
                    }
                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    //Now we rotate right to an angle somewhere between 90 and 180
                    ticks = 0;
                    if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = degreesToTicks(90);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        //We turn the robot 45 degrees to align with the center column
                        ticks = degreesToTicks(90);
                    } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = degreesToTicks(75);
                    }
                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARD;
                    break;
                case FORWARD:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(6), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.DONE;
                    break;
                case DONE:
                    isDone = true;
                    break;
            }

        } else if (side == Side3796.Blue2) {
            switch (state) {
                case WAIT:
                    if(driveOrder.isDone() || i > waitLimit) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    //First we drive forward 2 feet to get our robot off the platform
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(24), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    //Now we rotate right to an angle somewhere between 90 and 180
                    int ticks = 0;
                    if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = degreesToTicks(13.50);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        //We turn the robot 34 degrees to align with the center column
                        ticks = degreesToTicks(36);
                    } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = degreesToTicks(90);
                    }
                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARD;
                    break;
                case FORWARD:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(10), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(16), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    }
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.DONE;
                    if(vumark == RelicRecoveryVuMark.RIGHT){
                        stateAfterWait = NavigationState.ROTATE2;
                    }
                    break;
                case ROTATE2:
                    driveOrder = new JAWLDriveOrder3796(drive, degreesToTicks(85), JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARD2;
                    break;
                case FORWARD2:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(6), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.DONE;
                    break;
                case DONE:
                    isDone = true;
                    break;
            }


        }

        tele.addData("Nav state", state);
        tele.update();
    }

    enum NavigationState {
        WAIT, GETOFFPLATFORM, ROTATE, BACKWARD, ROTATE2, FORWARD, FORWARD2, DONE
    }

    private static int inchesToTicks(double inches) {
        /*
         * Ticks per revolution on an andymark 40 = 1120
         * Diameter of wheels = 9.5cm or 3.740157in
         *
         * 3.740157 * pi = 11.75
         * For every revolution our wheels travel 11.75 inches
         */

        double neededRevolutions = (inches - 2) / 11.75;
        int ticks = (int) (neededRevolutions * 1120);
        return ticks;
    }

    /*
     * Note from joe:
     * We could have used the internal IMU on the REV expansion hub to handle headings
     * It probably would have been much more accurate
     * However right now we don't have much time till the next competition
     */
    private static int degreesToTicks(double degrees){
        double temp = degrees * (1086 / 90);
        int ticks = (int)temp;
        return ticks;
    }
}