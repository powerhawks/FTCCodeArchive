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

@Autonomous(name = "AutoTestV13")
public class AutoTestV13 extends SkeletonOp {

    public static final String TAG = "Vuforia VuMark Sample";

    public String finalVuMark = "UNKNOWN";

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
//                    goGrab(-0.75, 1650);
//                    telemetry.addData("state", "goGrab");
//                    goSlide(.75,1200);
//                    telemetry.addData("state", "goSlide");
//                    deployJewlArm();
//                    Thread.sleep(2000);
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
//                    updateColorSensor();
//                    if (isBlue()){
//                        knockLeft(100);
//                        retractJewlArm();
//                        Thread.sleep(250);
//                        knockRight(100);
//                    }
//                    else if (isRed()){
//                        knockRight(100);
//                        retractJewlArm();
//                        Thread.sleep(250);
//                        knockLeft(100);
//                    }
//                    else{
//                        retractJewlArm();
//                        Thread.sleep(250);
//
//                    }
//
//                    Thread.sleep(400);
//                    retractJewlArm();



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

