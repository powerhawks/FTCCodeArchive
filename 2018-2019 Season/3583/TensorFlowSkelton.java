/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public abstract class TensorFlowSkelton extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Abdg8LX/////AAABmSf7RINGpU/jkpjLj3pF/N0RGUB6ns3w7MLkNCju9HKm4f1tgfE3Ya/IjpsCrd9UXKIlUZPES9za475tuCfCA5gCsmyGJLf64mgG1jj375x/B6fOvV3aTyvTH7oOO8Nd9SR903r9LWcmwS37WxMZSKzJrtek2WBcoWOzTVEe/Cx7gnkmh8SjcEMIf2W3AToIzZ01SNmo5W39vHxJ6vlhmbndSvpdQBL81PZpBvZVH0Jz9qZTB1F2Efrs1rfQSTLILTQ7Y9fSNZUdTVZD/sulu86uorlL35IvLJtuTeBe+1hpb5/zHgVPHZH3saBmR01tPBfDk6Kjp41lYvdC8on7W3xvJCUdqT1AS/hmf7fv2N/G"
            + "\n";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public String detectMinerals(boolean seesLeft) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    return "Left";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    return "Right";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    return "Center";
                                }
                            }
                        } else if (updatedRecognitions.size() == 2) {
                            String mineral1;
                            String mineral2;
                            if (seesLeft) {
                                if (updatedRecognitions.get(0).getLeft() > updatedRecognitions.get(1).getLeft()) {
                                    mineral1 = updatedRecognitions.get(0).getLabel();
                                    mineral2 = updatedRecognitions.get(1).getLabel();
                                } else {
                                    mineral1 = updatedRecognitions.get(1).getLabel();
                                    mineral2 = updatedRecognitions.get(0).getLabel();
                                }
                                if (mineral1.equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    return "Left";
                                } else if (mineral2.equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    return "Center";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    return "Right";
                                }
                            } else {
                                if (updatedRecognitions.get(0).getLeft() > updatedRecognitions.get(1).getLeft()) {
                                    mineral1 = updatedRecognitions.get(0).getLabel();
                                    mineral2 = updatedRecognitions.get(1).getLabel();
                                } else {
                                    mineral1 = updatedRecognitions.get(1).getLabel();
                                    mineral2 = updatedRecognitions.get(0).getLabel();
                                }
                                if (mineral1.equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    return "Center";
                                } else if (mineral2.equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    return "Center";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    return "Left";
                                }
                            }
                        }else if(updatedRecognitions.size() == 1){
                            if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                                return "Center";
                            }else{
                                return "Not Found";
                            }
                        }
//                      int numSilver = 0;
//                      int numGold = 0;
//                      for (Recognition recognition : updatedRecognitions) {
//                          telemetry.addLine(recognition.getLabel());
//                          telemetry.addData("Angle: ", recognition.estimateAngleToObject(AngleUnit.DEGREES));
//                          if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){ numGold++;}
//                          if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)){numSilver++;}
//                      }
//                      telemetry.addData("Number of Gold: ", numGold);
//                      telemetry.addData("Number of Silver: ", numSilver);
//                      telemetry.update();
//                    }

                    }
                }
                telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        return "Center";
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}