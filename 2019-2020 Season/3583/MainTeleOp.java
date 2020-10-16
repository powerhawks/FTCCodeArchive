package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "frame")
public class MainTeleOp extends TeleSkeletonSkystone {
    private static final double ACCEPTINPUTTHRESHOLD = 0.15;
    private boolean holding = false;
    private boolean robotCentric = true;
    public void runOpMode() {
        initRobot();
        while(!opModeIsActive()) {
            if (!Gyro.initialized) {
                initTeleGyro();
                OFFSET = 0;
                telemetry.addLine("Gyro not initialized, initializing gyro!");
            }
            if (gamepad1.dpad_up) {
                initTeleGyro();
                OFFSET = 0;
                telemetry.addLine("Init requested, initializing gyro!");
            }
            telemetry.update();
            waitForStart();
        }
        while (opModeIsActive()) {
            telemetry.addData("Holding", holding);
            if(robotCentric){
                telemetry.addLine("Drive System: Robot-Centric");
            }else{
                telemetry.addLine("Drive System: Field-Centric");
                telemetry.addData("Heading", getHeading());
            }
            if(driveDirection == -1){
                telemetry.addLine("Drive Direction: Inverted");
            }else{
                telemetry.addLine("Drive Direction: Direct");
            }
            telemetry.addData("Drive Speed", speed.getSpeed());
            telemetry.update();
            if(gamepad1.x){
                driveDirection = -1;
            }else if(gamepad1.y){
                driveDirection = 1;
            }
            if(gamepad1.left_trigger > 0.1){
                speed = DriveSpeed.HIGHPOWER;
            }else if(gamepad1.right_trigger > 0.1){
                speed = DriveSpeed.LOWPOWER;
            }else{
                speed = DriveSpeed.MIDPOWER;
            }
            double inputX, inputY, inputC;
            inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
            inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
            inputC = Math.abs(gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x : 0;
            if(robotCentric) {
                arcadeMecanum(-inputY, inputX, -inputC);
            }else{
                fieldCentric(inputY, inputX, -inputC);
            }

            if(gamepad1.dpad_up){
                robotCentric = true;
            }
            if(gamepad1.dpad_down){
                robotCentric = false;
            }
            if (gamepad1.dpad_left){
                OFFSET = 0;
                initTeleGyro();
            }


            //grab controls our grabber at the front of the robot for picking up cubes. This allows us to pick up cubes perindicular to our robot.
            if (gamepad2.left_bumper) {
                holding = false;
                grab.setPower(1);
            } else if (gamepad2.right_bumper) {
                holding = false;
                grab.setPower(-1);
            } else if (!holding) {
                grab.setPower(0);
            }
            if (gamepad2.a) {
                grab.setPower(1);
                holding = true;
            } else if (gamepad2.y) {
                grab.setPower(0);
                holding = false;
            }

            if (gamepad2.right_trigger > 0.1) {
                tapeMeasure.setPower(gamepad2.right_trigger);
            } else {
                tapeMeasure.setPower(-gamepad2.left_trigger);
            }

            while (gamepad1.a && gamepad2.a) {
                setPower(0);
            }

            leftLift.setPower(gamepad2.left_stick_y);
            rightLift.setPower(gamepad2.left_stick_y);

            if (drag.getPower() == 1 && gamepad2.dpad_down) {
                drag.setPower(-1);
            } else if (gamepad2.dpad_up) {
                drag.setPower(1);
            }
            //Plays Cybirds Screech whenever lift moves, commented o-ut for sanity
//            if (gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15) {
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
//            }
        }
    }
}