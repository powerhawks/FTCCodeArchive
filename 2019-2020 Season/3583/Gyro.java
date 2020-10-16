package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gyro{
    public static BNO055IMU imu;
    public static HardwareMap hardwareMap = null;
    static boolean initialized = false;
}
