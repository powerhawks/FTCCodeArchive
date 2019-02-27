package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Power Hawks Robotics on 12/14/2017.
 */

/*
introduced diagonal movements by altering the front wheel and back
wheel on the opposite side
 */

@Autonomous(name = "AutoTestMovements")
public class AutoTestV13Movement extends SkeletonOp {

    public static final String TAG = "Vuforia VuMark Sample";

    public String finalVuMark = "UNKNOWN";

    public boolean vuLoop = true;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

//    private boolean loopBreak = false;
//    i tried to create loop to do some color scanning but it didnt work so i deleted
//    public RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    public void initRobot(){
        GrabDrive = hardwareMap.dcMotor.get("grabber");
        SlideDrive = hardwareMap.dcMotor.get("slide");
        leftFrontWheel = hardwareMap.dcMotor.get("l1");
        leftBackWheel = hardwareMap.dcMotor.get("l2");
        rightFrontWheel = hardwareMap.dcMotor.get("r1");
        rightBackWheel = hardwareMap.dcMotor.get("r2");
        jewlArm = hardwareMap.servo.get("ja");
        sensorOfColor = hardwareMap.colorSensor.get("cs");
    }
    @Override
    public void runOpMode() throws InterruptedException{

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = RandomThings.vuKey;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //ToDo: Check and see if trackables is needed for just recognition.
        //
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        relicTrackables.activate();

        boolean autoMoveDone = false;

        try {
            while (opModeIsActive()) {
                if (!autoMoveDone) {

                    Thread.sleep(500);
                    if(vuLoop){
                        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                            telemetry.addData("VuMark", "%s visible", vuMark);
                            finalVuMark = vuMark.toString();
                        }
                        else {
                            telemetry.addData("VuMark", "not visible");
                            finalVuMark = vuMark.toString();
                        }
                        telemetry.addData("Phone Read:", finalVuMark);
                        telemetry.update();
                        relicTrackables.deactivate();
                        Thread.sleep(500);
                        vuLoop = false;
                    }


                    goGrab(-0.75, 1650);
                    telemetry.addData("state", "goGrab");
                    goSlide(.75,1200);
                    telemetry.addData("state", "goSlide");

                    deployJewlArm();



                    updateColorSensor();
                    if (isBlue()){
                        knockLeft(100);
                        retractJewlArm();
                        Thread.sleep(250);
                        knockRight(100);
                    }
                    else if (isRed()){
                        knockRight(100);
                        retractJewlArm();
                        Thread.sleep(250);
                        knockLeft(100);
                    }
                    else{
                        retractJewlArm();
                        Thread.sleep(250);

                    }
                    Thread.sleep(400);
                    retractJewlArm();

                    if (finalVuMark == "LEFT"){
                        goBackwards(0.5, 1400);
                        rotateRight(1, 720  );
                        goForward(.25, 1100);
                        goGrab(0.75, 300);
                    }
                    else if (finalVuMark == "CENTER"){
                        goBackwards(0.5, 1550);
                        rotateRight(1, 720  );
                        goForward(.25, 1100);
                        goGrab(0.75, 300);
                    }
                    else if (finalVuMark == "RIGHT"){
                        goBackwards(0.5, 1750);
                        rotateRight(1, 720  );
                        goForward(.25, 1100);
                        goGrab(0.75, 300);
                    }
                    else{
                        stopRobot();
                    }

                    goBackwards(.25,300);
                    goForward(.25, 350);
                    goBackwards(.25,350);
                    goGrab(.65, 1650);
                    goSlide(-0.65,1200);
                    autoMoveDone = true;


                }
                idle();
            }
        }
        finally {
            stopRobot();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

