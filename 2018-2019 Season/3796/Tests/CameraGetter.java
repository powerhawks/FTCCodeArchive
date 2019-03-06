package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.CameraImpl;

@Autonomous(name = "CamGet", group = "Qualifier")
@Disabled
public class CameraGetter extends LinearOpMode {

    VuforiaLocalizer vuforia;

    WebcamName webcamName;

    @Override public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Abdg8LX/////AAABmSf7RINGpU/jkpjLj3pF/N0RGUB6ns3w7MLkNCju9HKm4f1tgfE3Ya/IjpsCrd9UXKIlUZPES9za475tuCfCA5gCsmyGJLf64mgG1jj375x/B6fOvV3aTyvTH7oOO8Nd9SR903r9LWcmwS37WxMZSKzJrtek2WBcoWOzTVEe/Cx7gnkmh8SjcEMIf2W3AToIzZ01SNmo5W39vHxJ6vlhmbndSvpdQBL81PZpBvZVH0Jz9qZTB1F2Efrs1rfQSTLILTQ7Y9fSNZUdTVZD/sulu86uorlL35IvLJtuTeBe+1hpb5/zHgVPHZH3saBmR01tPBfDk6Kjp41lYvdC8on7W3xvJCUdqT1AS/hmf7fv2N/G"
            + "\n";

        parameters.cameraName = webcamName;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        waitForStart();

        while (opModeIsActive()) {}
    }
}