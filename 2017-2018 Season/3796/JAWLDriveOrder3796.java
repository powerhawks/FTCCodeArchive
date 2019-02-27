package org.firstinspires.ftc.teamcode;

/**
 * This class will help us manage the orders we give to the drive system
 */

public class JAWLDriveOrder3796 {
    private JAWLDrive3796 drive;
    private int ticks;
    private int[] oldPositions;
    private int[] targetPositions;
    private OrderType type;
    private int variance = 20;
    private double power;

    //ticks sould never be negative
    public JAWLDriveOrder3796(JAWLDrive3796 drive, int ticks, OrderType type, double power) {
        this.drive = drive;
        this.ticks = ticks;
        this.type = type;
        this.power = power;

        oldPositions = drive.getPositions();

        if(type == OrderType.FORWARD) {
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] + ticks};
        } else if (type == OrderType.BACKWARD) {
            targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] - ticks};
        } else if (type == OrderType.LEFTTURN) {
            targetPositions = new int[]{oldPositions[0] - ticks, oldPositions[1] + ticks};
        } else if (type == OrderType.RIGHTTURN) {
            targetPositions = new int[]{oldPositions[0] + ticks, oldPositions[1] - ticks};
        }

    }

    public void run() {
        if(type == OrderType.FORWARD) {
            drive.forward(ticks, power);
        } else if (type == OrderType.BACKWARD) {
            drive.backwards(ticks, power);
        } else if (type == OrderType.LEFTTURN) {
            drive.turnLeft(ticks, power);
        } else if (type == OrderType.RIGHTTURN) {
            drive.turnRight(ticks, power);
        }
    }

    public boolean isDone() {
        if(drive.isBusy(targetPositions[0], targetPositions[1], variance)) {
            return false;
        } else {
            return true;
        }
    }

    //This describes what type of order is given
    public enum OrderType {
        FORWARD, BACKWARD, LEFTTURN, RIGHTTURN
    }
}
