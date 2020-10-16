package org.firstinspires.ftc.teamcode.Delta_Robotics_9351_Autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.SkystonePatternPipelineRed;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous(name="Autonomo Skystone Azul", group="Final")
public class AutonomoSkystoneAzul extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private SkystonePatternPipelineRed patternPipeline;
    private WebcamName webcam;
    int pattern = 0;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //creamos la camara de OpenCV
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        //la inicializamos
        phoneCam.openCameraDevice();
        //creamos la pipeline
        patternPipeline = new SkystonePatternPipelineRed();

        //definimos la pipeline para la camara
        phoneCam.setPipeline(patternPipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("[/!\\]", "Recuerden posicionar correctamente el robot, con los dos rectangulos que se ven en la camara apuntando justo hacia las dos ultimas stones de la quarry (las mas cercanas a el skybridge)\n\nGOOO DELTA ROBOTICS!!!");
        telemetry.update();

        //esperamos que el usuario presione <play> en la driver station
        waitForStart();
/*
        Thread t;
        t = new Thread(){
            public void run(){
                while(opModeIsActive()) {
                    status.opModeIsActive = true;
                }
                status.opModeIsActive = false;
            }
        };
        t.start();
*/
        if(patternPipeline.pattern == 0){
            while(opModeIsActive());
        }

        pattern = patternPipeline.pattern;

        //phoneCam.closeCameraDevice(); //apagamos la camara ya que no es necesaria a partir de este punto.

        telemetry.addData("Pattern", pattern); //mandamos mensaje telemetry para reportar que ya se detecto un patron
        telemetry.addData("Left", SkystonePatternPipelineRed.valLeft);
        telemetry.addData("Right", SkystonePatternPipelineRed.valRight);
        telemetry.update();
        sleep(10000);


        if(pattern == 2){ //este falta el ultimo skystone

        }else if(pattern == 3){ //este ya esta

        }else if(pattern == 1){

        }else{
            //en teoria este codigo nunca se deberia de ejecutar, pero por si las dudas...
            telemetry.addData("[ERROR]", "No se que ha pasado ni como has llegado hasta aqui. Lo siento =(");
            telemetry.update();
            while(opModeIsActive());
        }
    }

}