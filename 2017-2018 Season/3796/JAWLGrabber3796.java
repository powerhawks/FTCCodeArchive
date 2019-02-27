package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Power Hawks Robotics on 11/14/2017.
 *
 * November 14, 2017
 * Chase Galey and Joe Lewis
 * Tony the Train That Tried Glyph Grabber Methods
 */

public class JAWLGrabber3796 {

    private Servo leftArm;
    private Servo rightArm;


    //Most of this was guessing and checking to find the correct values
    //DO NOT CHANGE THESE UNLESS THE ORIENTATION OF THE GRABBER ARMS WAS CHANGED
    //Turns out the mounting method we used for the arms allowed them to slip during game if they were hit
    //We fixed that but now we have new values for open and closed
    //The old values are commented out below

    /*
    The positions for each of the arms are different because of the way they were attached
    The 0.7 is the position to keep a funnel for the glyphs
    The 0.5 is the position to snugly hold the glyphs when lifting and/or moving
    double leftPositionClosed = 0.5;
    double leftPositionOpen = 0.2;
    double rightPositionClosed = 0.2;
    double rightPositionOpen = 0.5;
    */

    /* Old values
    double leftPositionClosed = 0.75;
    double leftPositionOpen = 0.5;
    double rightPositionClosed = 0.5;
    double rightPositionOpen = 0.7;*/

    double leftPositionClosed = 0.7;
    double leftPositionOpen = 0.4;
    double rightPositionClosed = 0.5;
    double rightPositionOpen = 0.8;
    double leftShortOpen = .575;
    double rightShortOpen = .625;

    public JAWLGrabber3796(Servo left, Servo right){

        //Sets the names of the servo
        this.leftArm = left;
        this.rightArm = right;
    }

    /**
     * Because of the way that the two servos were placed on the robot,
     * we had to switch the controls for one of the arms so that the
     * bumpers would close them both and the triggers would open them.
     */
    public void rightArmShort(){
        //Closes the right arm less
        rightArm.setPosition(rightShortOpen);
    }

    public void leftArmShort(){
        //Closes the left arm less
        leftArm.setPosition(leftShortOpen);
    }

    public void rightArmClose(){
        //Closes the right arm
        rightArm.setPosition(rightPositionClosed);
    }

    public void rightArmOpen(){
        //Opens the right arm
        rightArm.setPosition(rightPositionOpen);
    }

    public void leftArmClose(){
        //Closes the left arm
        leftArm.setPosition(leftPositionClosed);
    }

    public void leftArmOpen(){
        //Opens the left arm
        leftArm.setPosition(leftPositionOpen);
    }
    /*
    public void leftArmMove(double position){
        double positionNew = position;
//        if(position <= 0.45) {
//            positionNew = 0.45;
//        }

        double x = 1 - positionNew;
        double y = leftPositionClosed + x;
        positionNew = y;

        if(positionNew >= leftPositionOpen) {
            positionNew = leftPositionOpen;
        }

        leftArm.setPosition(positionNew);
    }

    public void rightArmMove(double position){
        double positionNew = position;
//        if(position <= 0.45) {
//            positionNew = 0.45;
//        }

        double x = 1 - positionNew;
        double y = rightPositionClosed + x;
        positionNew = y;

        if(positionNew >= rightPositionOpen) {
            positionNew = rightPositionOpen;
        }

        rightArm.setPosition(positionNew);
    }
    */

    public double rightPosition(){
        return rightArm.getPosition();
    }

    public double leftPosition(){
        return leftArm.getPosition();
    }

}