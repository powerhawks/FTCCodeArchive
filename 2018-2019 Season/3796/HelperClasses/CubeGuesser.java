package org.firstinspires.ftc.teamcode.HelperClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enums.MaterialOrder;

import java.util.List;
import java.util.Random;

/**
 * Name: Cube Guesser
 * Authors: Lincoln Doney
 * Team: FTC Talons 3796 2018-2019 season
 * Date: ?
 *
 * This code is written up so that the robot still has a chance of getting the order of the cube correct
 * even if the camera cannot see all of the cubes. This essentially gives our autonomous a better success
 * rate rather than just taking the raw input
 */

@Disabled
public class CubeGuesser {
    //Sets all of the Labels for TensorFlow
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Guess() method... This is static because we don't want to initialize a CubeGuesser object every
    //time we want to guess something
    public static MaterialOrder guess(List<Recognition> updatedRecognitions) {
        //Get a random number (Just as a worst case scenario)
        Random r = new Random();
        int guessNum = r.nextInt(updatedRecognitions.size());

        //Set all of the positions to -1 (Meaning unknown)
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;
        //Loop through the recognitions to get their position for comparison later
        for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                //Set the gold mineral
                goldMineralX = (int) recognition.getLeft();
            } else if (silverMineral1X == -1) {
                //Set the 1st silver mineral
                silverMineral1X = (int) recognition.getLeft();
            } else {
                //Set the second silver mineral
                silverMineral2X = (int) recognition.getLeft();
            }
        }

        //If it can see all three, then it runs the standard algorithm and returns the exact amount.
        //This is a best case scenario, and technically is not really needed in this method but it's
        //good to have just in case
        if (updatedRecognitions.size() == 3) {
            //Make sure none of the minerals are set to -1... if they are then that's an oopsie because
            //that means the code above just didn't work and that shouldn't be programatically possible
            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                    //If the gold is on the left, return it being on the left
                    return MaterialOrder.Left;
                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                    //If the gold is on the right, return it being on the right
                    return MaterialOrder.Right;
                } else {
                    //If the gold is in the center, return it being on the center
                    return MaterialOrder.Center;
                }
            }
        //If the gold can only see two, then it runs guesses. This is based on the camera being on the
        //right, so most likely the right two minerals are being seen, and so the code compensates for that.
        } else if (updatedRecognitions.size() == 2) {
            if (silverMineral1X != -1 && silverMineral2X != -1) {
                //No gold found but both silvers found.. Return left
                return MaterialOrder.Left;
            } else if ((silverMineral1X != -1 || silverMineral2X != -1) && goldMineralX != -1) {
                //One silver found and one gold found
                if (((silverMineral1X < goldMineralX) && silverMineral1X != -1) ||
                        (silverMineral2X < goldMineralX) && silverMineral2X != -1) {
                    //Silver on left of gold.. Return right
                    return MaterialOrder.Right;
                } else if (((silverMineral1X > goldMineralX) && silverMineral1X != -1) ||
                        ((silverMineral2X > goldMineralX) && silverMineral2X != -1)) {
                    //Silver on right of gold.. Return center
                    return MaterialOrder.Center;
                }
            }
        //If it can only see one... then the code just takes a completely random guess (This is worst
        //case scenario and means that the robot has no clue the order).. This is 100% random and can
        //either work flawlessly or run the wrong part of the autonomous. At this point it's just up
        //to luck.
        } else if (updatedRecognitions.size() == 1) {
            if (guessNum == 0) {
                //If the guess is zero, return left
                return MaterialOrder.Left;
            } else if (guessNum == 1) {
                //If the guess is 1, return center
                return MaterialOrder.Center;
            } else if (guessNum == 2) {
                //If the guess is 2, return right
                return MaterialOrder.Right;
            }
            //Again this is just 100% random
        } else if (updatedRecognitions.size() == 0) {
            //If it just doesn't see anything then it returns that it's unknown. This will run the
            //center code, so it still has a 33% chance of being right.
            return MaterialOrder.Unknown;
        } else {
            //Same as the code above, it's just a just-in-case statement
            return MaterialOrder.Unknown;
        }
            //Again, another just-in-case
        return MaterialOrder.Unknown;
    }
}
