//package org.firstinspires.ftc.teamcode.Concepts;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//@Disabled
//@TeleOp (name = "OpenCV Test")
//public class OpenCV_Test extends OpMode {
//
//    private StoneAnalyzer stoneAnalyzer = new StoneAnalyzer();
//    private OpenCvCamera camera;
//
//    @Override
//    public void init() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        camera.openCameraDevice();
//        camera.setPipeline(stoneAnalyzer);
//
//        // The next line starts the camera and the StoneAnalyzer `processFrame` will start getting called in its own thread.
//        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//    }
//
//    @Override
//    public void loop() {
//        int passingPixels = stoneAnalyzer.getNumPassingPixels();
//        double percentagePassing = 100.0 * passingPixels / (320.0 * 240.0); // 100.0 to convert the ratio to a percentage. 320 * 240 = total number of pixels.
//        telemetry.addData("Percentage passing", percentagePassing);
//    }
//}
