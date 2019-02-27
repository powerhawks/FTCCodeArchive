package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/*
 * Created by Power Hawks Robotics on 9/26/2017.
 * Joe and Chase
 *
 * Created for team 3796 to prototype our grabber design
 * Only controls two servos with one controller
 *
 * Updated from 3.1 to 3.4 on 10/5/2017
 * Somethings got screwed up so we had
 * to change and fix. That is the main
 * reason why there are so many commented
 * lines.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Prototype: Servo Tester")
@Disabled
public class OldGrabberCode extends LinearOpMode {

//    static final double INCREMENT   =  0.5;     // amount to slew servo each CYCLE_MS cycle
//    static final int    CYCLE_MS    =   50;     // period of each cycle
//    static final double MAX_POS     =  1.0;     // Maximum rotational position
//    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define servos
    Servo leftArm;
    Servo rightArm;
    //Define doubles for keeping track of arm postions
    double leftPosition;
    double rightPosition;
    //Arm closed open
    double positionOpen = 0.5;
    //Arm open position
    double positionClosed = 0;
    //double newpositionOpen = 1.0;
    //double newpositionClosed = 0.75;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
//    boolean rampUp = true;


    @Override
    public void runOpMode() {
        //Set starting positions to 0.75
        //Assumes opmode will start with arms closed
        leftPosition = positionClosed;
        rightPosition = positionClosed;

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftArm = hardwareMap.servo.get("left_arm");
        rightArm = hardwareMap.servo.get("right_arm");

        leftArm.setPosition(positionClosed);
        leftPosition = positionClosed;

        rightArm.setPosition(positionOpen);
        rightPosition = positionOpen;

        // Wait for the start button
        telemetry.addData(">", "Press Start to start prototype");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
/*            if (rampUp) {
*                // Keep stepping up until we hit the max value.
*                position += INCREMENT ;
*                if (position >= MAX_POS ) {
*                    position = MAX_POS;
*                    rampUp = !rampUp;   // Switch ramp direction
*                }
*            }
*            else {
*                // Keep stepping down until we hit the min value.
*                position -= INCREMENT ;
*                if (position <= MIN_POS ) {
*                    position = MIN_POS;
*                    rampUp = !rampUp;  // Switch ramp direction
*                }
*            }
*/

//             if (gamepad2.b) {
//                if (rightPosition == positionOpen) {
//                    rightArm.setPosition(positionClosed);
//                    rightPosition = positionClosed;
//                }
//            }
//
//            if (gamepad2.a) {
//
//                if (rightPosition == positionClosed) {
//                    rightArm.setPosition(positionOpen);
//                    rightPosition = positionOpen;
//                }
//
//            }
//
//            if (gamepad2.x) {
//                if (leftPosition == positionOpen) {
//                    leftArm.setPosition(positionClosed);
//                    leftPosition = positionClosed;
//                }
//            }
//
//            if (gamepad2.y){
//
//                if (leftPosition == positionClosed) {
//                    leftArm.setPosition(positionOpen);
//                    leftPosition = positionOpen;
//                }
//
//            }





            if (gamepad2.right_trigger > 0){

                if (rightPosition == positionOpen) {
                    rightArm.setPosition(positionClosed);
                    rightPosition = positionClosed;
                }

            }

            if (gamepad2.left_bumper){

                if (leftPosition == positionOpen) {
                    leftArm.setPosition(positionClosed);
                    leftPosition = positionClosed;
                }

            }

            if (gamepad2.right_bumper){

                if (rightPosition == positionClosed) {
                    rightArm.setPosition(positionOpen);
                    rightPosition = positionOpen;
                }

            }

            if (gamepad2.left_trigger > 0){

                if (leftPosition == positionClosed) {
                    leftArm.setPosition(positionOpen);
                    leftPosition = positionOpen;
                }

            }

            // Display the current value
            telemetry.addData("Right Arm Position", "%5.2f", rightPosition);
            telemetry.addData("Left Arm Position", "%5.2f", leftPosition);
            telemetry.update();

            // Set the servo to the new position and pause;
            //servo.setPosition(position);
            //sleep(CYCLE_MS);

            //sleep(500);
        }

        // Signal done;
        //telemetry.addData(">", "Done");
        //telemetry.update();
    }
}