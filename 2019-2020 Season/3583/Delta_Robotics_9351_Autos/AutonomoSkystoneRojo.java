package org.firstinspires.ftc.teamcode.Delta_Robotics_9351_Autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pipelines.SkystonePatternPipelineBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous(name="Autonomo Skystone Rojo", group="Final")
public class AutonomoSkystoneRojo extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private SkystonePatternPipelineBlue patternPipeline;
    //private IMUDriveMecanum imuTurn;
    int pattern = 0;

    //public OpModeStatus status = new OpModeStatus(false);

    @Override
    public void runOpMode() {
        //hdw = new Hardware(hardwareMap);
        //hdw.initHardware(false);

        //imuTurn = new IMUDriveMecanum(hdw, telemetry, status);
        //timeDrive = new TimeDriveMecanum(hdw, telemetry);

        //imuTurn.initIMU();

        telemetry.addData("[/!\\]", "Calibrando el sensor IMU, espera...");

        telemetry.update();
        //obtenemos la id del monitor de la camara (la vista de la camara que se vera desde el robot controller)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //creamos la camara de OpenCV
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //la inicializamos
        phoneCam.openCameraDevice();

        //creamos la pipeline
        patternPipeline = new SkystonePatternPipelineBlue();

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

        status.opModeIsActive = true;
        */
        //si el pattern es 0 (si es 0 significa que no ha detectado ningun pattern) simplemente nos estacionaremos debajo del skybridge
        if(patternPipeline.pattern == 0){
            while(opModeIsActive());
        }

        pattern = patternPipeline.pattern;

        phoneCam.closeCameraDevice(); //apagamos la camara ya que no es necesaria a partir de este punto.

        telemetry.addData("Pattern", pattern); //mandamos mensaje telemetry para reportar que ya se detecto un patron
        telemetry.update();

        sleep(2000);

        if(pattern == 1){

        }else if(pattern == 2){

        }else if(pattern == 3){

        }else{
            //en teoria este codigo nunca se deberia de ejecutar, pero por si las dudas...
            telemetry.addData("[ERROR]", "No se que ha pasado ni como has llegado hasta aqui. Lo siento =(");
            telemetry.update();
            while(opModeIsActive());
        }
    }

}