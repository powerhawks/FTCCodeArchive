package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enums.KowallskiSide3796;

/**
 * Name: Depot side of Autonomous
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: Febuary 19, 2019
 * */
@Autonomous(name="Autonomous Depot", group="Final")
public class KowallskiAutonomousDepot3796 extends KowallskiAutonomousDropDown3796 {

    @Override
    public void runOpMode() throws InterruptedException {
        //Set side variable of autonomous to depot
        super.side = KowallskiSide3796.Depot;
        //Run the base autonomous
        super.runOpMode();
    }
}
