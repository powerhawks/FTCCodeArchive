package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * January 20, 2018
 * Chase Galey and Joe Lewis
 *
 * Class created to test our new navigation method
 */

@Autonomous(name = "Navigation Test", group = "tests")
public class NavigationTest extends LinearOpMode {
    public void runOpMode() {
        JAWLDrive3796 drive = new JAWLDrive3796(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        drive.setEncoders(true);

        double power = 0.7;
        boolean isDone = false;
        NavState state = NavState.FORWARDS;
        NavState stateAfterWait = NavState.WAIT;
        JAWLDriveOrder3796 driveOrder = null;
        int i = 0;

        waitForStart();

        while(opModeIsActive() && !isDone) {
            switch (state) {
                case WAIT:
                    telemetry.addData("Drive is done", driveOrder.isDone());
                    telemetry.addData("I", i);
                    if(driveOrder.isDone()) {
                        if(i > 40) {
                            state = stateAfterWait;
                            i = 0;
                        } else {
                            i++;
                        }
                    }
                    break;
                case FORWARDS:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(10), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavState.WAIT;
                    stateAfterWait = NavState.TURNRIGHT;
                    break;
                case TURNRIGHT:
                    driveOrder = new JAWLDriveOrder3796(drive, 1086, JAWLDriveOrder3796.OrderType.RIGHTTURN, power);
                    driveOrder.run();
                    state = NavState.WAIT;
                    stateAfterWait = NavState.FORWARDS2;
                    break;
                case FORWARDS2:
                    driveOrder = new JAWLDriveOrder3796(drive, inchesToTicks(10), JAWLDriveOrder3796.OrderType.FORWARD, power);
                    driveOrder.run();
                    state = NavState.WAIT;
                    stateAfterWait = NavState.DONE;
                    break;
                case DONE:
                    isDone = true;
                    break;

            }

            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    enum NavState {
        WAIT, FORWARDS, TURNRIGHT, FORWARDS2, DONE
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
