package org.firstinspires.ftc.teamcode.Rover_Ruckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public abstract class ThotimusSkeltonTeleOpv7 extends OpMode {

    private DcMotor motor;
    public DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, scoopLift, liftMechanism, collection;
    public Servo parkBar;
    public BNO055IMU imu = null;

    //Mecanum Drive
    //    4 motorS
    static final String LEFT1NAME = "l1";
    static final String LEFT2NAME = "l2";
    static final String RIGHT1NAME = "r1";
    static final String RIGHT2NAME = "r2";

    //Lifting Mechanism
    //      1 motor
    static final String LIFTNAME = "lift";

    //Intake
    //    1 motor
    //    1 crservo
    static final String SCOOPLIFT = "sLift";
    static final String SCOOP = "scoop";
    static final String COLLECT = "collect";

    static final String GYRO = "gyro";

    //this method is the base drive and mathematics for our mecanum algorithmic drive
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        c = -c;
        x = -x;

        double leftFrontVal = -y - x + c;
        double rightFrontVal = -y + x - c;

        // try setting these equal to their front most counterpart
        double leftBackVal = -y + x + c;
        double rightBackVal = -y - x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        //double scaledPower = gearShift[gear];
        double scaledPower = 1;
        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower)); //sets to 1
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower)); //sets to -1
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower)); //sets to -1
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower)); //sets to 1
    }

    //Emergency method to cut power to all components
    public static void stopAll(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, DcMotor lift, DcMotor iLift, DcMotor collection){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        lift.setPower(0);
        iLift.setPower(0);
        collection.setPower(0);

    }

    public void init(){
        //Mechanum wheels
        motor = hardwareMap.dcMotor.get("motor");
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);

        //Lift mechanism
        liftMechanism = hardwareMap.dcMotor.get(LIFTNAME);


        collection = hardwareMap.dcMotor.get(COLLECT);

        scoopLift = hardwareMap.dcMotor.get(SCOOPLIFT);
        scoopLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parkBar = hardwareMap.servo.get("parkBar");
        parkBar.setPosition(1);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void logMotors() {
        int lf, lb, rf, rb;
        double lfp, lbp, rfp, rbp;
        lf = leftFrontWheel.getCurrentPosition();
        lb = leftBackWheel.getCurrentPosition();
        rf = rightFrontWheel.getCurrentPosition();
        rb = rightBackWheel.getCurrentPosition();
        lfp = leftFrontWheel.getPower();
        lbp = leftBackWheel.getPower();
        rfp = rightFrontWheel.getPower();
        rbp = rightBackWheel.getPower();
        telemetry.addData("Left Front Wheel Position: ", lf);
        telemetry.addData("Left Back Wheel Position: ", lb);
        telemetry.addData("Right Front Wheel Posiiton: ", rf);
        telemetry.addData("Right Back Wheel Posiiton: ", rb);
        telemetry.addData("Left Front Wheel Power: ", lfp);
        telemetry.addData("Left Back Wheel Power: ", lbp);
        telemetry.addData("Right Front Wheel Power: ", rfp);
        telemetry.addData("Right Back Wheel Power: ", rbp);

    }

}
