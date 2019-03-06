/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Enums.diag;

public class FarquadGyroTest {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     TURN_SPEED              = 0.5;

    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;

    DcMotor rightFrontWheel, rightBackWheel, leftFrontWheel, leftBackWheel;
    ModernRoboticsI2cGyro gyro    = null;

    public void resetHeaders()
    {
        stopMovement();
        gyro.calibrate();
        gyro.resetZAxisIntegrator();
    }
    public FarquadGyroTest(DcMotor rightFront, DcMotor rightBack, DcMotor leftFront, DcMotor leftBack, ModernRoboticsI2cGyro gyro)
    {
        this.gyro = gyro;
        this.rightFrontWheel = rightFront;
        this.rightBackWheel = rightBack;
        this.leftFrontWheel = leftFront;
        this.leftBackWheel = leftBack;
    }
    public FarquadGyroTest(DcMotor rightFront, DcMotor rightBack, DcMotor leftFront, DcMotor leftBack){

        this.rightFrontWheel = rightFront;
        this.rightBackWheel = rightBack;
        this.leftFrontWheel = leftFront;
        this.leftBackWheel = leftBack;

    }
    static final double powerScalar = 0.5;
    //These are the most basic methods in this class.
    //They will be called by each of the

    public void moveFrontRightWheel(double power){
        rightFrontWheel.setPower(power * powerScalar);
    }

    public void moveBackRightWheel(double power){
        rightBackWheel.setPower(power * powerScalar);
    }

    public void moveFrontLeftWheel(double power){
        leftFrontWheel.setPower(power * powerScalar);
    }

    public void moveBackLeftWheel(double power){
        leftBackWheel.setPower(power * powerScalar);
    }

    public void forwardBackward(double power)
    {

        //When you move the joystick the other way, it will reverse the direction of movement, so we do not need to write two methods for this.
        this.moveBackLeftWheel((power));
        this.moveBackRightWheel((-power));
        this.moveFrontLeftWheel((power));
        this.moveFrontRightWheel((-power));
    }
    public void leftRight(double power)
    {

        //When you move the joystick the other way, it will reverse the direction of movement, so we do not need to write two methods for this.
        this.moveBackLeftWheel(-(power * powerScalar));
        this.moveBackRightWheel(-(power * powerScalar));
        this.moveFrontLeftWheel((power * powerScalar));
        this.moveFrontRightWheel((power * powerScalar));
    }
    public void diagonal(diag dir)
    {
        switch(dir)
        {
            //These will check which enum dir has been set to. We only have two motors move because of the way that Mecanum wheels work.
            case upleft:
                //Moving the FrontRight wheel and the BackLeft wheel forwards will make the robot slide diagonally front-left because of how the Mecanum wheels work.
                this.moveBackLeftWheel((powerScalar));
                this.moveFrontRightWheel(-(powerScalar));
                break;
            case upright:
                //Moving the FrontLeft wheel and the BackRight wheel forwards will make the robot slide diagonally front-right because of the Mecanum wheels.
                this.moveBackRightWheel(-(powerScalar));
                this.moveFrontLeftWheel((powerScalar));
                break;
            case downleft:
                //Moving the FrontLeft wheel and the BackRight wheel backwards will make the robot slide diagonally back-left because of the way the Mecanum wheels are made.
                this.moveBackRightWheel((powerScalar));
                this.moveFrontLeftWheel(-(powerScalar));
                break;
            case downright:
                //Moving the FrontRight wheel and the BackLeft wheel backwards will make the robot slide diagonally back-right because of the angle of the Mecanum wheels.
                this.moveBackLeftWheel(-(powerScalar));
                this.moveFrontRightWheel((powerScalar));
                break;
            default:
                break;
        }
    }

    public void turn(double power){

        //Applying the same power to all the motors will make them all spin the same direction, clockwise or counterclockwise.
        //Since two of them are in the opposite orientation, this will make the robot spin.

        moveBackLeftWheel(power * powerScalar);
        moveBackRightWheel(power * powerScalar);
        moveFrontLeftWheel(power * powerScalar);
        moveFrontRightWheel(power * powerScalar);

    }

    //If we need to make sure that no motors are moving at all what so ever, we can use this.

    public void stopMovement()
    {
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
        leftFrontWheel.setPower(0);
    }

    public void setEncoders(boolean bool){
        if(bool) {
            rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void resetEncoders() {
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = leftFrontWheel.getCurrentPosition() + moveCounts;
        newRightTarget = rightFrontWheel.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftFrontWheel.setTargetPosition(newLeftTarget);
        rightFrontWheel.setTargetPosition(newLeftTarget);

        leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftFrontWheel.setPower(speed);
        leftBackWheel.setPower(speed);
        rightFrontWheel.setPower(speed);
        rightBackWheel.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy())) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftFrontWheel.setPower(leftSpeed);
            leftBackWheel.setPower(leftSpeed);
            rightFrontWheel.setPower(rightSpeed);
            rightBackWheel.setPower(rightSpeed);
        }

        stopMovement();

        setEncoders(true);
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        stopMovement();
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftBackWheel.setPower(leftSpeed);
        leftFrontWheel.setPower(leftSpeed);
        rightBackWheel.setPower(rightSpeed);
        rightFrontWheel.setPower(rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
*/