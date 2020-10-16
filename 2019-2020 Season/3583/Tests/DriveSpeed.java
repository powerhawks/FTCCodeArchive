package org.firstinspires.ftc.teamcode.Tests;

enum DriveSpeed {
    LOWPOWER(.25), MIDPOWER(.75), HIGHPOWER(1.0);

    private double speed;

    private DriveSpeed(double speed){
        this.speed = speed;
    }

    double getSpeed(){
        return speed;
    }
}
