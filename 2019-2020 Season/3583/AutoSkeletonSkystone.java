package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Pipelines.SkystonePatternPipelineRed;
import org.firstinspires.ftc.teamcode.Pipelines.SkystonePatternPipelineBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class AutoSkeletonSkystone extends LinearOpMode {

    //Components
	DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, tapeMeasure;
	DcMotorEx leftLift, rightLift;
	CRServo grab, drag;
	private BNO055IMU imu = null;
	ElapsedTime runtime = new ElapsedTime();

	//Camera
	OpenCvCamera phoneCam;

	//Tracks alliance color
	boolean red = true;

	//Pipelines
	SkystonePatternPipelineRed patternPipelineRed;
	SkystonePatternPipelineBlue patternPipelineBlue;

	//Tracks what orientation the quarry is in
	public int pattern = 0;

	//Co+stants for movment
	final int DELAY = 444;
	final double POWER = .75;
	final double TURNPOWER = .30;
	final double FASTTURNPOWER = 0.75;
	final int QUARTERTURN = 91;
	final double STRAFECORRECTIONRATIO = 0.12;
	static final double     P_DRIVE_COEFF           = 0.15;



	//Variables for wheels and accurate movement
	//28 * 20 / (2ppi * 4.125)
	private final int COUNTS_PER_ROTATION = 28; //counts per rotation
	private final int GEAR_RATIO = 20;
	private final double DIAMETER = 2.95276;
	private final double COUNTS_PER_INCH = (COUNTS_PER_ROTATION * GEAR_RATIO)/(Math.PI * DIAMETER); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
	private final double BIAS = 1.0;//default 0.8
	private final double STRAFING_BIAS = 1.0;//change to adjust only strafing movement
	private final double CONVERSION = COUNTS_PER_INCH * BIAS;
	private boolean exit = false;

	//Variables for gyro turning
	private Orientation angles;
	private Acceleration gravity;
	private static final double HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro
	private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
	private static final double CORRECTION_POWER = 1.0f;

	//Constant for moving lift with encoders
	private final double ticksPerInchLift = 66;

	//Open the claw and scan the quarry from starting  position to determine orientation of skystones
	void findPattern(){
		grab.setPower(-1);
		if(red) {
			pattern = patternPipelineRed.pattern;
			phoneCam.closeCameraDevice();
		}else{
			pattern = patternPipelineBlue.pattern;
			phoneCam.closeCameraDevice();
		}
	}

	//Move over the first skystone depending on the pattern
	void moveToFirstSkystone(){
		moveToPosition(10, POWER);
		if(red){
			if(pattern == 1){ //Positions 1 and 4
//				strafeToPosition(7, POWER);
				DriveRightCorrect(7);
				grab.setPower(0);
				sleep(DELAY);
			}else if(pattern == 2){ // Positions 0 and 3
//				strafeToPosition(-3, POWER);
				DriveLeftCorrect(3);
				grab.setPower(0);
				sleep(DELAY);
			}else if(pattern == 3){ // Positions 2 and 5
//				strafeToPosition(16, POWER);
				DriveRightCorrect(16);
				grab.setPower(0);
				sleep(DELAY);
			}else{ //Cannot detect
//				strafeToPosition(7, POWER);
				DriveRightCorrect(7);
				grab.setPower(0);
			}
			DriveForwardCorrect(14.5);
			sleep(DELAY);
		}else{
			if(pattern == 1){ //Positions 1 and 4
				DriveLeftCorrect(8);
				grab.setPower(0);
				gyroHold(TURNPOWER, 0, DELAY/1000);
			}else if(pattern == 2){ // Positons 0 and 3
				DriveLeftCorrect(16.5);
				grab.setPower(0);
				gyroHold(TURNPOWER, 0 , DELAY/1000);
			}else if(pattern == 3){ // Positions 2 and 5
				DriveRightCorrect(2.5);
				grab.setPower(0);
				gyroHold(TURNPOWER, 0, DELAY/1000);
			}else{ //Cannot detect
				DriveLeftCorrect(7);
				grab.setPower(0);
				gyroHold(TURNPOWER, 0, DELAY/1000);
			}
			DriveForwardCorrect(16);
			gyroHold(TURNPOWER, 0, 1);
		}
//		moveToPosition(15, POWER);

	}

	//Picks up a skystone if the robot is directly in front of it
	void pickUpSkystone(){
		setLiftPosition(-4, 1);
		sleep(DELAY);
		grab.setPower(1);
		sleep(1500);
	}

	//Move to the foundation after picking up the first (rightmost) skystone
	void moveToFoundationFromFirstSkystone(){
		moveToPosition(-6, POWER);
		sleep(DELAY);
		if(red) {
			turnWithGyro(90, TURNPOWER);
			sleep(DELAY);			//Strafe a given distance + some other distance depending on where the skystone was
			if(pattern == 1){
//				moveToPosition(80, POWER);
				DriveForwardCorrect(80);
				sleep(DELAY);
			}else if(pattern == 2){
//				moveToPosition(90, POWER);
				DriveForwardCorrect(90);
				sleep(DELAY);
			}else if(pattern == 3){
//				moveToPosition(72, POWER);
				DriveForwardCorrect(72);
				sleep(DELAY);
			}else{
//				moveToPosition(80, POWER);
				DriveForwardCorrect(80);
				sleep(DELAY);
			}
			turnWithGyro(QUARTERTURN, -TURNPOWER);
			sleep(DELAY);
		}else{
			//Strafe a given distance + some other distance depending on where the ksystone was
			if(pattern == 1){
				turnWithGyro(90, -TURNPOWER);
				sleep(DELAY);
				DriveForwardCorrect(85);
				sleep(DELAY);
			}else if(pattern == 2){
				turnWithGyro(90, -TURNPOWER);
				sleep(DELAY);
				DriveForwardCorrect(75);
				sleep(DELAY);
			}else if(pattern == 3){
				turnWithGyro(92d, -TURNPOWER);
				sleep(DELAY);
				DriveForwardCorrect(90);
				sleep(DELAY);
			}else{
				DriveForwardCorrect(80);
				sleep(DELAY);
			}
			turnWithGyro(90, TURNPOWER);
		}
		setLiftPosition(6.5, 1);
		sleep(DELAY);
		DriveForward(9);
	}

	void moveToFoundationFromSecondSkystone(){
		moveToPosition(-10, POWER);
		sleep(DELAY);
		if(red) {
			turnWithGyro(90, -TURNPOWER);
			sleep(DELAY);
			//Strafe a given distance + some other distance depending on where the skystone was
			if(pattern == 1){
//				moveToPosition(-100, POWER);
//				DriveBackwardCorrect(100);
				moveToPosition(-80, POWER);
				sleep(DELAY);
			}else if(pattern == 2){
//				moveToPosition(90, POWER);
				DriveBackwardCorrect(90);
				sleep(DELAY);
			}else if(pattern == 3){
//				moveToPosition(72, POWER);
				DriveBackwardCorrect(72);
				sleep(DELAY);
			}else{
//				moveToPosition(80, POWER);
				DriveBackwardCorrect(100);
				sleep(DELAY);
			}
		}else{
			//Strafe a given distance + some other distance depending on where the ksystone was
			if(pattern == 1){
				DriveLeft(43);
				DriveLeft(43);
			}else if(pattern == 2){
				strafeToPosition(-16, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(-16, POWER);
				sleep(DELAY);
			}else if(pattern == 3){
				strafeToPosition(-45, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(-45, POWER);
				sleep(DELAY);
			}else{
				strafeToPosition(-43, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(-43, POWER);
				sleep(DELAY);
			}
			turnWithGyro(QUARTERTURN, -TURNPOWER);
		}
	}

	void moveUnderBridgeFromFirstSkystone(){
		moveToPosition(-5, POWER);
		sleep(DELAY);
		if(red) {
			//Strafe a given distance + some other distance depending on where the skystone was
			if(pattern == 1){
//				strafeToPositionCorrect(60, POWER);
//				DriveRightCorrect(80);
				strafeToPositionCorrect(50, POWER);
			}else if(pattern == 2){
//				DriveRight(45);
//				DriveRight(45);
				DriveRightCorrect(45);
			}else if(pattern == 3){
//				DriveRight(20);
//				DriveRight(20);
				DriveRightCorrect(40);
			}else{
//				DriveRight(24);
//				DriveRight(24);
				DriveRightCorrect(60);
			}
		}else{
			//Strafe a given distance + some other distance depending on where the ksystone was
			if(pattern == 1){
				DriveLeft(28);
				DriveLeft(28);
			}else if(pattern == 2){
				DriveLeft(24);
				DriveLeft(24);
			}else if(pattern == 3){
				DriveLeft(32);
				DriveLeft(32);
			}else{
				DriveLeft(28);
				DriveLeft(28);
			}
		}
	}

	//Drops a skystone into the foundation if the robot is directly in front of it
	void depositSkystone(){
		if(red) {
			grab.setPower(-1);
			sleep(500);
			grab.setPower(0);
			strafeToPosition(-12, POWER);
			sleep(DELAY);
		}else{
			grab.setPower(-1);
			sleep(500);
			grab.setPower(0);
			strafeToPosition(17, POWER);
			sleep(DELAY);
		}
	}

	void depositSkystoneUnderBridge(){
		setLiftPosition(4, 1);
//		moveToPosition(10, POWER);
		DriveForwardCorrect(10);
		grab.setPower(-1);
		sleep(500);
		grab.setPower(1);
//		moveToPosition(-10, POWER);
		DriveBackwardCorrect(10);
		setLiftPosition(-4, 1);
	}

	void moveToSecondSkystone(){
		if(red) {
			//Strafe a given distance + some other distance depen    ding on where the skystone was
			if(pattern == 1){
//				strafeToPositionCorrect(-80, POWER);
//				DriveLeftCorrect(80);
				strafeToPosition(-78, POWER);
				gyroHold(.2, 0, 1.25);
			}else if(pattern == 2){
//				DriveLeft(45);
//				DriveLeft(45);
				DriveLeftCorrect(100);
			}else if(pattern == 3){
//				DriveLeft(20);
//				DriveLeft(20);
				DriveLeftCorrect(70);
			}else{
//				DriveLeft(24);
//				DriveLeft(24);
				DriveLeftCorrect(80);
			}
		}else{
			//Strafe a given distance + some other distance depending on where the ksystone was
			if(pattern == 1){
				strafeToPosition(34, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(34, POWER);
				sleep(DELAY);
				DriveRight(34);
				DriveRight(34);
			}else if(pattern == 2){
				strafeToPosition(30, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(30, POWER);
				sleep(DELAY);
			}else if(pattern == 3){
				strafeToPosition(38, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(38, POWER);
				sleep(DELAY);
			}else{
				strafeToPosition(34, POWER);
				gyroHold(TURNPOWER, 0, DELAY);
				strafeToPosition(34, POWER);
				sleep(DELAY);
			}
		}
		grab.setPower(-1);
		setLiftPosition(4, 1);
		sleep(DELAY);
		//moveToPosition(10, POWER);
		//sleep(DELAY);
	}

	//After depositing the second skystone, drag the foundation back into the building site
	void moveFoundation(){
		if(red){
			drag.setPower(-1);
			sleep(DELAY);
			strafeToPosition(-35, POWER);
			sleep(DELAY);
			turnWithGyro(110, TURNPOWER);
			sleep(DELAY);
			drag.setPower(1);
			sleep(DELAY);
			turnWithGyro(80, -TURNPOWER);
		}else{
			drag.setPower(-1);
			sleep(DELAY);
			strafeToPosition(-31, POWER);
			sleep(DELAY);
			turnWithGyro(90, -TURNPOWER);
			sleep(DELAY);
			drag.setPower(1);
			sleep(2*DELAY);
			turnWithGyro(75, -TURNPOWER);
		}
	}

	void depositySingleSkystone(){
		if(red){
			if(pattern == 1){
				moveToPosition(40, POWER);
				sleep(DELAY);
			}else if(pattern == 2){
				moveToPosition(45, POWER);
				sleep(DELAY);
			}else if(pattern == 3){
				moveToPosition(35, POWER);
				sleep(DELAY);
			}else{
				moveToPosition(40, POWER);
				sleep(DELAY);
			}
		}else{
			if(pattern == 1){
				moveToPosition(45, POWER);
				sleep(DELAY);
			}else if(pattern == 2){
				moveToPosition(40, POWER);
				sleep(DELAY);
			}else if(pattern == 3){
				moveToPosition(50, POWER);
				sleep(DELAY);
			}else{
				moveToPosition(45, POWER);
				sleep(DELAY);
			}
		}
		moveToPosition(-10, POWER);
	}

	public void initRobot() {
	    //Initialize all components
		leftFrontWheel = hardwareMap.dcMotor.get("left_front");
		leftBackWheel = hardwareMap.dcMotor.get("left_back");
		rightFrontWheel = hardwareMap.dcMotor.get("right_front");
		rightBackWheel = hardwareMap.dcMotor.get("right_back");
//		leftLift = hardwareMap.dcMotor.get("left_lift");
//		rightLift = hardwareMap.dcMotor.get("right_lift");
		leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
		rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");
		tapeMeasure = hardwareMap.dcMotor.get("tape_measure");
        grab = hardwareMap.crservo.get("grab");
        drag = hardwareMap.crservo.get("grabber");
		//Reverse some wheels because we need to do that
		rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
		leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//Set the drag up to fir inside the sizing cube and prevent it from getting in the way
		drag.setPower(1);
	}

	public void initWebcam() {
		//Camera components
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		phoneCam.openCameraDevice();
			patternPipelineRed = new SkystonePatternPipelineRed();
			patternPipelineBlue = new SkystonePatternPipelineBlue();
			if(red) {
				phoneCam.setPipeline(patternPipelineRed);
			}else{
				phoneCam.setPipeline(patternPipelineBlue);
			}
		phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
	}
	//move both lift motors at once to set the lift to some position
	void setLiftPosition(double inches, double power){
		leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		double ticksLift = inches * ticksPerInchLift;
		leftLift.setTargetPosition(leftLift.getCurrentPosition() - (int)ticksLift);
		rightLift.setTargetPosition(rightLift.getCurrentPosition() - (int)ticksLift);
		leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftLift.setPower(power);
		rightLift.setPower(power);
		while(leftLift.isBusy() && rightLift.isBusy()){
			telemetry.addLine(leftLift.getCurrentPosition() + "/" + leftLift.getTargetPosition());
			telemetry.addLine(rightLift.getCurrentPosition() + "/" + rightLift.getTargetPosition());
			telemetry.update();
		}
		leftLift.setPower(0);
		rightLift.setPower(0);
	}

	private void setMode(DcMotor.RunMode x){
		leftFrontWheel.setMode(x);
		rightFrontWheel.setMode(x);
		leftBackWheel.setMode(x);
		rightBackWheel.setMode(x);
	}

	private void setPower(double power){
		leftFrontWheel.setPower(power);
		rightFrontWheel.setPower(power);
		leftBackWheel.setPower(power);
		rightBackWheel.setPower(power);
	}

	private boolean isBusy(){
		return leftBackWheel.isBusy() && leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy();
	}


	public void DriveLeft(double inches) {
		encoderDrive(POWER, inches, -inches, inches, -inches, 10000);
	}

	public void DriveLeftCorrect(double inches){
		encoderDriveCorrect(POWER, inches, -inches, inches, -inches, 10000);
	}

	public void DriveRight(double inches) {
		encoderDrive(POWER, -inches, inches, -inches, inches, 10000);
	}

	public void DriveRightCorrect(double inches) {
		encoderDriveCorrect(POWER, -inches, inches, -inches, inches, 10000);
	}

	public void DriveForward(double inches){
		encoderDrive(POWER, inches, inches, inches, inches, 1500);
	}

	public void DriveForwardCorrect(double inches){
		encoderDriveCorrect(POWER, inches, inches, inches, inches, 1500);
	}

	public void DriveBackwardCorrect(double inches){
		encoderDriveCorrect(POWER, -inches, -inches, -inches, -inches, 10000);
	}

	public void encoderDrive(double speed,
							 double leftBackInches,
							 double leftFrontInches,
							 double rightFrontInches,
							 double rightBackInches,
							 double timeoutS) {
		int newLeftFrontTarget, newRightBackTarget, newLeftBackTarget, newRightFrontTarget;
		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			newLeftBackTarget = leftBackWheel.getCurrentPosition() + ((int)(Math.round(leftBackInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newRightBackTarget = rightBackWheel.getCurrentPosition() + ((int)(Math.round(rightBackInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newLeftFrontTarget = leftFrontWheel.getCurrentPosition() + ((int)(Math.round(leftFrontInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newRightFrontTarget = rightFrontWheel.getCurrentPosition() + ((int)(Math.round(rightFrontInches * COUNTS_PER_INCH * STRAFING_BIAS)));

			leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			leftBackWheel.setTargetPosition(newLeftBackTarget);
			rightFrontWheel.setTargetPosition(newRightFrontTarget);
			rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();

			setPower(Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(isBusy())) {

				telemetry.addLine("LF: " + leftFrontWheel.getCurrentPosition() + "/" + leftFrontWheel.getTargetPosition());
				telemetry.addLine("LB: " + leftBackWheel.getCurrentPosition() + "/" + leftBackWheel.getTargetPosition());
				telemetry.addLine("RF: " + rightFrontWheel.getCurrentPosition() + "/" + rightFrontWheel.getTargetPosition());
				telemetry.addLine("RB: " + rightBackWheel.getCurrentPosition() + "/" + rightBackWheel.getTargetPosition());
				telemetry.update();
			}

			// Stop all motion;
			setPower(0);

			// Turn off RUN_TO_POSITION
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			gyroHold(.3, 0, 1);
			// optional pause after each move
		}
	}

	public void encoderDriveCorrect(double speed,
							 double leftBackInches,
							 double leftFrontInches,
							 double rightFrontInches,
							 double rightBackInches,
							 double timeoutS) {
		int newLeftFrontTarget, newRightBackTarget, newLeftBackTarget, newRightFrontTarget;
		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			double startYaw = angles.firstAngle;

			// Determine new target position, and pass to motor controller
			newLeftBackTarget = leftBackWheel.getCurrentPosition() + ((int)(Math.round(leftBackInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newRightBackTarget = rightBackWheel.getCurrentPosition() + ((int)(Math.round(rightBackInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newLeftFrontTarget = leftFrontWheel.getCurrentPosition() + ((int)(Math.round(leftFrontInches * COUNTS_PER_INCH * STRAFING_BIAS)));
			newRightFrontTarget = rightFrontWheel.getCurrentPosition() + ((int)(Math.round(rightFrontInches * COUNTS_PER_INCH * STRAFING_BIAS)));

			leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			leftBackWheel.setTargetPosition(newLeftBackTarget);
			rightFrontWheel.setTargetPosition(newRightFrontTarget);
			rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();

			setPower(Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(isBusy())) {

				telemetry.addLine("LF: " + leftFrontWheel.getCurrentPosition() + "/" + leftFrontWheel.getTargetPosition());
				telemetry.addLine("LB: " + leftBackWheel.getCurrentPosition() + "/" + leftBackWheel.getTargetPosition());
				telemetry.addLine("RF: " + rightFrontWheel.getCurrentPosition() + "/" + rightFrontWheel.getTargetPosition());
				telemetry.addLine("RB: " + rightBackWheel.getCurrentPosition() + "/" + rightBackWheel.getTargetPosition());

				double angle = startYaw;
				double  error;
				double  steer;
				error = getError(angle);
				steer = getSteer(error, P_DRIVE_COEFF);
				double speedL = speed - (steer / 7);
				double speedR = speed + (steer / 7);


				// Normalize speeds if either one exceeds +/- 1.0;
				double max = Math.max(Math.abs(speedL), Math.abs(speedR));
				if (max > 1.0)
				{
					speedL /= max;
					speedR /= max;
				}

				leftFrontWheel.setPower(speedL);
				leftBackWheel.setPower(speedL);
				rightFrontWheel.setPower(speedR);
				rightBackWheel.setPower(speedR);

				telemetry.addLine("Steer: " + steer);
				telemetry.update();
			}

			// Stop all motion;
			setPower(0);

			// Turn off RUN_TO_POSITION
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            gyroHold(speed, startYaw, 0.1);
			// optional pause after each move
		}
	}

	public void gyroHold(double speed, double angle, double holdTime) {

		ElapsedTime holdTimer = new ElapsedTime();

		// keep looping while we have time remaining.
		holdTimer.reset();
		while (opModeIsActive() && (holdTimer.time() < holdTime)) {
			// Update telemetry & Allow time for other processes to run.
			onHeading(speed, angle, P_TURN_COEFF);
			telemetry.update();
		}

		// Stop all motion;
		setPower(0);
	}

	/**
	 * Perform one cycle of closed loop heading control.
	 *
	 * @param speed  Desired speed of turn.
	 * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
	 *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *               If a relative angle is required, add/subtract from current heading.
	 * @param PCoeff Proportional Gain coefficient
	 * @return
	 */
	boolean onHeading(double speed, double angle, double PCoeff) {
		double error;
		double steer;
		boolean onTarget = false;
		double leftSpeed;
		double rightSpeed;

		// determine turn power based on +/- error
		error = getError(angle);

		if (Math.abs(error) <= HEADING_THRESHOLD) {
			steer = 0.0;
			leftSpeed = 0.0;
			rightSpeed = 0.0;
			onTarget = true;
		} else {
			steer = getSteer(error, PCoeff);
			rightSpeed = speed * steer * CORRECTION_POWER;
			leftSpeed = -rightSpeed;
		}

		// Send desired speeds to motors.
		leftBackWheel.setPower(leftSpeed);
		leftFrontWheel.setPower(leftSpeed);
		rightBackWheel.setPower(rightSpeed);
		rightFrontWheel.setPower(rightSpeed);

		// Display it for the driver.
		telemetry.addData("Target", "%5.2f", angle);
		telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
		telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

		return onTarget;
	}

	/**
	 * getError determines the error between the target angle and the robot's current heading
	 *
	 * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
	 * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
	 * +ve error means the robot should turn LEFT (CCW) to reduce error.
	 */
	public double getError(double targetAngle) {

		double robotError;

		// calculate error in -179 to +180 range  (
		angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double yaw = angles.firstAngle;

		robotError = targetAngle - yaw;
		while (robotError > 180) robotError -= 360;
		while (robotError <= -180) robotError += 360;
		return robotError;
	}

	/**
	 * returns desired steering force.  +/- 1 range.  +ve = steer left
	 *
	 * @param error  Error angle in robot relative degrees
	 * @param PCoeff Proportional Gain Coefficient
	 * @return
	 */
	public double getSteer(double error, double PCoeff) {
		return Range.clip(error * PCoeff, -1, 1);
	}

	void strafeToPositionCorrect(double inches, double speed){
		//
		int move = (int)(Math.round(inches * COUNTS_PER_INCH * STRAFING_BIAS));
		//
		leftBackWheel.setTargetPosition(leftBackWheel.getCurrentPosition() - move);
		leftFrontWheel.setTargetPosition((int) (leftFrontWheel.getCurrentPosition() + (move + STRAFECORRECTIONRATIO*move)));
		rightBackWheel.setTargetPosition(rightBackWheel.getCurrentPosition() + move);
		rightFrontWheel.setTargetPosition((int) (rightFrontWheel.getCurrentPosition() - (move + STRAFECORRECTIONRATIO*move)));
		//
		setMode(DcMotor.RunMode.RUN_TO_POSITION);
		//
		leftFrontWheel.setPower(speed);
		rightFrontWheel.setPower(speed);
		leftBackWheel.setPower(speed-(speed*STRAFECORRECTIONRATIO));
		rightBackWheel.setPower(speed-(speed*STRAFECORRECTIONRATIO));
		//
		while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){}
		setPower(0);
	}











	/*
	This function's purpose is simply to drive forward or backward.
	To drive backward, simply make the inches input negative.
	 */
	void moveToPosition(double inches, double speed){
		//
		int move = (int)(Math.round(inches*CONVERSION));
		//
		leftBackWheel.setTargetPosition(leftBackWheel.getCurrentPosition() + move);
		leftFrontWheel.setTargetPosition(leftFrontWheel.getCurrentPosition() + move);
		rightBackWheel.setTargetPosition(rightBackWheel.getCurrentPosition() + move);
		rightFrontWheel.setTargetPosition(rightFrontWheel.getCurrentPosition() + move);
		//
		setMode(DcMotor.RunMode.RUN_TO_POSITION);
		//
		setPower(speed);
		//
		while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){
			if (exit){
				setPower(0);
				return;
			}
		}
		setPower(0);
	}
	//
	/*
	This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
	Degrees should always be positive, make speedDirection negative to turn left.
	 */
	public void turnWithGyro(double degrees, double speedDirection){
		//<editor-fold desc="Initialize">
		angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double yaw = -angles.firstAngle;//make this negative
		telemetry.addData("Speed Direction", speedDirection);
		telemetry.addData("Yaw", yaw);
		telemetry.update();
		//
		telemetry.addData("stuff", speedDirection);
		telemetry.update();
		//
		double first;
		double second;
		//</editor-fold>
		//
		if (speedDirection > 0){//set target positions
			//<editor-fold desc="turn right">
			if (degrees > 10){
				first = (degrees - 10) + devertify(yaw);
				second = degrees + devertify(yaw);
			}else{
				first = devertify(yaw);
				second = degrees + devertify(yaw);
			}
			//</editor-fold>
		}else{
			//<editor-fold desc="turn left">
			if (degrees > 10){
				first = devertify(-(degrees - 10) + devertify(yaw));
				second = devertify(-degrees + devertify(yaw));
			}else{
				first = devertify(yaw);
				second = devertify(-degrees + devertify(yaw));
			}
			//
			//</editor-fold>
		}
		//
		//<editor-fold desc="Go to position">
		Double firsta = convertify(first - 5);//175
		Double firstb = convertify(first + 5);//-175
		//
		turnWithEncoder(speedDirection);
		//
		if (Math.abs(firsta - firstb) < 11) {
			while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				gravity = imu.getGravity();
				yaw = -angles.firstAngle;
				telemetry.addData("Position", yaw);
				telemetry.addData("first before", first);
				telemetry.addData("first after", convertify(first));
				telemetry.update();
			}
		}else{
			//
			while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				gravity = imu.getGravity();
				yaw = -angles.firstAngle;
				telemetry.addData("Position", yaw);
				telemetry.addData("first before", first);
				telemetry.addData("first after", convertify(first));
				telemetry.update();
			}
		}
		//
		Double seconda = convertify(second - 5);//175
		Double secondb = convertify(second + 5);//-175
		//
		turnWithEncoder(speedDirection / 3);
		//
		if (Math.abs(seconda - secondb) < 11) {
			while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				gravity = imu.getGravity();
				yaw = -angles.firstAngle;
				telemetry.addData("Position", yaw);
				telemetry.addData("second before", second);
				telemetry.addData("second after", convertify(second));
				telemetry.update();
			}
			while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				gravity = imu.getGravity();
				yaw = -angles.firstAngle;
				telemetry.addData("Position", yaw);
				telemetry.addData("second before", second);
				telemetry.addData("second after", convertify(second));
				telemetry.update();
			}
			setPower(0);
		}
	}
	/*
	This function uses the encoders to strafe left or right.
	Negative input for inches results in left strafing.
	 */
	void strafeToPosition(double inches, double speed){
		//
		int move = (int)(Math.round(inches * COUNTS_PER_INCH * STRAFING_BIAS));
		//
		leftBackWheel.setTargetPosition(leftBackWheel.getCurrentPosition() - move);
		leftFrontWheel.setTargetPosition(leftFrontWheel.getCurrentPosition() + move);
		rightBackWheel.setTargetPosition(rightBackWheel.getCurrentPosition() + move);
		rightFrontWheel.setTargetPosition(rightFrontWheel.getCurrentPosition() - move);
		//
		setMode(DcMotor.RunMode.RUN_TO_POSITION);
		//
		setPower(speed);
		//
		while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){}
		setPower(0);
	}

	//
	/*
	These functions are used in the turnWithGyro function to ensure inputs
	are interpreted properly.
	 */
	private double devertify(double degrees){
		if (degrees < 0){
			degrees = degrees + 360;
		}
		return degrees;
	}
	private double convertify(double degrees){
		if (degrees > 179){
			degrees = -(360 - degrees);
		} else if(degrees < -180){
			degrees = 360 + degrees;
		} else if(degrees > 360){
			degrees = degrees - 360;
		}
		return degrees;
	}
	//
	/*
	This function is called at the beginning of the program to activate
	the IMU Integrated Gyro.
	 */
	public void initGyro(){
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		//parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		//
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
	}

	void initTeleGyro(){
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		//parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		//
		Gyro.imu = hardwareMap.get(BNO055IMU.class, "imu");
		Gyro.imu.initialize(parameters);
		Gyro.initialized = true;
	}
	//
	/*
	This function is used in the turnWithGyro function to set the
	encoder mode and turn.
	 */
	private void turnWithEncoder(double input){
		setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//
		leftFrontWheel.setPower(input);
		leftBackWheel.setPower(input);
		rightFrontWheel.setPower(-input);
		rightBackWheel.setPower(-input);
	}
}


