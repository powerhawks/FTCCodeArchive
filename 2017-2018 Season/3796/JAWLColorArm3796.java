package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * November 18, 2017
 * Chase Galey and Joe Lewis
 * Color arm component class
 *
 * Has methods for controlling the arm during autonomous and teleop
 */

public class JAWLColorArm3796 {
    private Servo colorArm;

    //The highest position it should be is 0.65
    //The lowest is 0.15
    private double positionUp = 0.65;
    private double positionDown = 0.13;

    //Sets the name for the color arm
    public JAWLColorArm3796(Servo arm) {
        colorArm = arm;
    }

    //Moves the arm up
    public void armUp() {
        colorArm.setPosition(positionUp);
    }

    //Moves the arm down
    public void armDown() {
        colorArm.setPosition(positionDown);
    }
}
