package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Name: Crater side of Autonomous
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: November 17, 2018
 * */

public class KowallskiCollectionServos3796 {

    //Initialize the left and right servos
    //Technically these aren't actually servos but they act as servos, they're really motors
    CRServo rightServo, leftServo;

    //Constructor for the collection servos.. Get these by the hardware map
    public KowallskiCollectionServos3796(CRServo right, CRServo left){
        this.rightServo = right;
        this.leftServo = left;
    }

    //Collect or Eject the minerals
    // (+) -> Collect
    // (-) -> Eject
    public void collectOrEject(double power){
        rightServo.setPower(power * 0.75);
        leftServo.setPower(power * 0.75);
    }

    //Stop the movement of the servos
    public void stopMovement(){

        rightServo.setPower(0);
        leftServo.setPower(0);

    }

}
