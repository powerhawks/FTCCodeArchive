package org.firstinspires.ftc.teamcode;

enum DriveSpeed {
    LOWPOWER(.25), MIDPOWER(.5), HIGHPOWER(.75);

    private double speed;

    private DriveSpeed(double speed){
        this.speed = speed;
    }

    double getSpeed(){
        return speed;
    }
}
