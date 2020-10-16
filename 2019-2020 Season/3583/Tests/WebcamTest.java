package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoSkeletonSkystone;
import org.firstinspires.ftc.teamcode.Pipelines.SkystonePatternPipelineCenter;
@Disabled
@Autonomous (name = "Webcam Test")
public class WebcamTest extends AutoSkeletonSkystone {
	@Override
	public void runOpMode() throws InterruptedException {
		SkystonePatternPipelineCenter pipeline = new SkystonePatternPipelineCenter();
		initWebcam();
		waitForStart();
		while (opModeIsActive()){
			pipeline.seesStone();
			telemetry.addData("Sees?", pipeline.seesSkystone);
			telemetry.update();
		}
	}
}