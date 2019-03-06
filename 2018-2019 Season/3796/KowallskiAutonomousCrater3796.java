package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enums.KowallskiSide3796;
/**
 * Name: Crater side of Autonomous
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: Febuary 19, 2019
 * */
@Autonomous(name="Autonomous Crater", group="Final")
public class KowallskiAutonomousCrater3796 extends KowallskiAutonomousDropDown3796 {

    @Override
    public void runOpMode() throws InterruptedException {
            //Set the side variable of autonomous to Crater
            super.side = KowallskiSide3796.Crater;
            //Run the base autonomous (What the class is derived off of)
            super.runOpMode();
    }
}
