package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "ThotimusPrimev8", group = "Current")
public class ThotimusTeleOpv8 extends ThotimusSkeltonTeleOpv7 {

    private static final double ACCEPTINPUTTHRESHOLD = 0.15;
    public Orientation angles;

    public void loop(){
        logMotors();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Angle:", angles.firstAngle);
        telemetry.addData("Lift value: ", liftMechanism.getCurrentPosition());
        telemetry.addData("Scoop Lift value: ", scoopLift.getCurrentPosition());
        telemetry.addData("inputY: ", gamepad1.left_stick_y);
        telemetry.addData("inputX: ", gamepad1.left_stick_x);
        telemetry.addData("inputC: ", gamepad1.right_stick_x);
        telemetry.addData("Servo Position: ", parkBar.getPosition());
        telemetry.update();


        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y: 0;
        double inputC = Math.abs(gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;
        arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        if(gamepad2.right_bumper){
            liftMechanism.setPower(-1);
        }else if(gamepad2.left_bumper){
            liftMechanism.setPower(1);
        }else {
            liftMechanism.setPower(0);
        }
        scoopLift.setPower(gamepad2.left_stick_y);
        collection.setPower(gamepad2.right_stick_y);

        while(gamepad1.a && gamepad2.a){
            stopAll(leftFrontWheel,rightFrontWheel,leftBackWheel,rightBackWheel,liftMechanism,scoopLift,collection);
        }

        if(gamepad2.dpad_down){
            parkBar.setPosition(parkBar.getPosition()-.05);
        }

        if(gamepad2.dpad_up){
            parkBar.setPosition(parkBar.getPosition()+.05);
        }
    }
}
