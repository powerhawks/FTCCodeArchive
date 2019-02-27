package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Power Hawks Robotics on 11/21/2017.
 * <p>
 * Chase Galey and Joe Lewis
 * November 21, 2017
 * Component Classes for the Color Sensors we will use
 * We are planning on using REV Color Distance Sensors
 */

public class JAWLColorSensor3796 {

    ColorSensor revColor;
    DistanceSensor revDistance;

    //Sets the range for the color ranges
    float MAX_BLUE = 270;
    float MIN_BLUE = 170;
    float MAX_RED = 332;
    float MIN_RED = 42;

    public JAWLColorSensor3796(ColorSensor color, DistanceSensor distance) {
        this.revColor = color;
        this.revDistance = distance;
    }

    public double getDistance() {
        return revDistance.getDistance(DistanceUnit.CM);
    }

    public String getColor() {
      float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        //Converts the RGB values to HSV values because the color is passive, not active
        Color.RGBToHSV((int) (revColor.red() * SCALE_FACTOR),
                (int) (revColor.green() * SCALE_FACTOR),
                (int) (revColor.blue() * SCALE_FACTOR),
                hsvValues);

//        int red = ((int) (revColor.red() * SCALE_FACTOR));
//        int blue = ((int) (revColor.blue() * SCALE_FACTOR));
//        int green = ((int) (revColor.green() * SCALE_FACTOR));

        float hue = hsvValues[0];
        float sat = hsvValues[1];
        float val = hsvValues[2];

        /*
        if(hue > 215 && hue < 260) {
            return hue;
        } else {
            return hue;
        }
        */

        String redW = "RED";
        String blueW = "BLUE";
        String other = "UR BAD";

        if(hue < MAX_BLUE && hue > MIN_BLUE){
            return blueW;
        }

        //Red goes from about 332 to 42, going over 360
        //So we had to swap the greater than and less than
        if(hue > MAX_RED || hue < MIN_RED){
            return redW;
        }else{
            return other;
        }

        /*
        if(green > red && green > blue) {

            return greenW;

        }else if(red > blue) {

            return redW;

        }else if(blue > red){

            return blueW;

        }else{

            return "null";

        }
        */
    }
}
