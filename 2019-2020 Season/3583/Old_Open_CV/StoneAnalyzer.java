package org.firstinspires.ftc.teamcode.Old_Open_CV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicInteger;
@Disabled
public class StoneAnalyzer extends OpenCvPipeline {

    // Rectangles are made using x, y, width, and height in pixels.
    // (x,y) is the top-left corner of the Rectangle
    // (x,y) = (0,0) is the top-left corner of the screen
    private Rect stoneRegion = new Rect(72, 185, 53, 27);

    // Scalar values can represent nearly anything.
    // In this case, it's a Hue, Saturation, and Value (respectively).
    // LOWER_HSV and UPPER_HSV are used to make an HSV filter.
    // This library uses 0 <= Hue < 180, and 0 <= Saturation/Value < 255
    private final Scalar LOWER_HSV = new Scalar(15.0, 120.0, 120.0);
    private final Scalar UPPER_HSV = new Scalar(35.0, 255.0, 255.0);

    // I use an AtomicInteger because it's faster than using Synchronized functions.
    // However, I provided a commented-out version at the bottom of this program that shows what you
    // would do if there isn't an "Atomic" data type for whatever you want to send back.
    private AtomicInteger numPassingPixels = new AtomicInteger(0);

    @Override
    public Mat processFrame(Mat input) {
        // Store what size the original image was.
        Size imgSize = input.size();

        // Grab a specific region of the image
        Mat focusRegion = input.submat(stoneRegion);
        // The "destination" of OpenCV functions cannot be null, so `new Mat()` must be called at a minimum.
        Mat hsvFocusRegion = new Mat();
        // Convert `focusRegion` to HSV and store the result in `hsvFocusRegion`
        Imgproc.cvtColor(focusRegion, hsvFocusRegion, Imgproc.COLOR_RGB2HSV);

        // It is also valid to put the same variable as the source AND destination if it's fine for OpenCV to overwrite the original data.
        Core.inRange(hsvFocusRegion, LOWER_HSV, UPPER_HSV, hsvFocusRegion); // Overwrite hsvFocusRegion
        // Atomic set/get is thread-safe
        numPassingPixels.set(Core.countNonZero(hsvFocusRegion));
        // What you would do instead if you were using a regular int:
        // setPassingPixels(Core.countNonZero(hsvFocusRegion)); // Calls the synchronized function at the bottom of this program

        // This Scalar represents color, either in RGB or BGR, I can't remember at the moment.
        // Either way, (0,255,0) = Green. The other two channels are Red and Blue
        Scalar color = new Scalar(0.0, 255.0, 0.0);
        // Draw box around the focus region. This will be displayed on the Robot Controller
        Imgproc.rectangle(input, stoneRegion, color, 1);

        // Make sure you output the same image size as the original.
        // This isn't needed here since we don't resize the input, but if you do, this is required.
        Imgproc.resize(input, input, imgSize);
        return input;
    }

    public int getNumPassingPixels() {
        return numPassingPixels.get();
    }

    /*
    // What you would use instead of the getter function above if you were using a regular int:
    // Two functions with the `synchronized` keyword won't run at the same time.
    // This prevents a thread from changing the value of numPassingPixels while another thread reads it.
    public synchronized int getNumPassingPixels() {
        return numPassingPixels;
    }

    public synchronized void setNumPassingPixels(int numPassingPixels) {
        this.numPassingPixels = numPassingPixels;
    }
     */
}