package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Power Hawks Robotics on 1/18/2018.
 */

public class OldOldAutonomousNavigation {

    //This is the power value we will use for all drive orders within this class
    private static double power = 0.7;
    //We initalize a state enum
    private static NavigationState state = NavigationState.GETOFFPLATFORM;
    //We initalize an enum we want to set after the wait is done
    private static NavigationState stateAfterWait = NavigationState.WAIT;
    private static JAWLDriveOrder3796 driveOrder = null;
    public static boolean isDone = false;
    private static int globalWaitInt = 40;
    private static int i = 0;

    public static void navigateToCryptobox(Side3796 side, RelicRecoveryVuMark vumark, JAWLDrive3796 drive, Telemetry tele) {
        if(vumark == RelicRecoveryVuMark.UNKNOWN) {
            tele.addLine("No Vumark found! Defaulting to center collumn");
            vumark = RelicRecoveryVuMark.CENTER;
        }

        if(side == Side3796.Red1) {



            switch (state) {
                case WAIT:
                    if(driveOrder.isDone()) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = inchesToTicks(32.185);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(39.815);
                    } else if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = inchesToTicks(47.445);
                    }

                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.BACKWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDSINTOBOX;
                    break;
                case FORWARDSINTOBOX:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(14), JAWLDriveOrder3796.OrderType.FORWARD, power);
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
                    if(driveOrder.isDone()) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(24), JAWLDriveOrder3796.OrderType.BACKWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDS;
                    break;
                case FORWARDS:
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = inchesToTicks(3.815);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(11.445);
                    } else if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = inchesToTicks(19.075);
                    }

                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE2;
                    break;
                case ROTATE2:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDSINTOBOX;
                    break;
                case FORWARDSINTOBOX:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(14), JAWLDriveOrder3796.OrderType.FORWARD, power);
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
                    if(driveOrder.isDone()) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = inchesToTicks(32.185);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(39.815);
                    } else if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = inchesToTicks(47.445);
                    }

                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDSINTOBOX;
                    break;
                case FORWARDSINTOBOX:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(14), JAWLDriveOrder3796.OrderType.FORWARD, power);
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
                    if(driveOrder.isDone()) {
                        if(i > globalWaitInt) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case GETOFFPLATFORM:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(24), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE;
                    break;
                case ROTATE:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDS;
                    break;
                case FORWARDS:
                    int ticks = 0;
                    if(vumark == RelicRecoveryVuMark.RIGHT) {
                        ticks = inchesToTicks(3.815);
                    } else if (vumark == RelicRecoveryVuMark.CENTER) {
                        ticks = inchesToTicks(11.445);
                    } else if (vumark == RelicRecoveryVuMark.LEFT) {
                        ticks = inchesToTicks(19.075);
                    }

                    driveOrder = new JAWLDriveOrder3796(drive, ticks, JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.ROTATE2;
                    break;
                case ROTATE2:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.LEFTTURN, power);
                    driveOrder.run();
                    state = NavigationState.WAIT;
                    stateAfterWait = NavigationState.FORWARDSINTOBOX;
                    break;
                case FORWARDSINTOBOX:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(14), JAWLDriveOrder3796.OrderType.FORWARD, power);
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
        WAIT, GETOFFPLATFORM, ROTATE, FORWARDSINTOBOX, FORWARDS, ROTATE2, DONE
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
}
