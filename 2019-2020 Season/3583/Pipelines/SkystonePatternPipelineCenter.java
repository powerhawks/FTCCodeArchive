package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystonePatternPipelineCenter extends OpenCvPipeline {

    //el funcionamiento de esta pipeline consiste en detectar las dos ultimas stones de la derecha de
    //el quarry y determinar con estas el pattern, y a partir de este pattern seguir instrucciones
    //especificas de movimiento para el robot, ya que conocemos la posicion de los skystones.

    //se aplica un filtro en el que las stones normales tienen un color diferente
    //a las skystones y asi se puede determinar si se trata de un skystone.


    //para mover los rectangulos (las zonas en las que se detecta el color) usa las siguientes variables y
    //no tendras que mover nada mas. La posicion es en relacion al tamano de la vista de la camara

    private static final float rectangleX = 10f;

    private static final float rectangleY = 5.3f;


    //en teoria no hay necesidad de tocar nada a partir de aqui.
    public static int valLeft = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;
    private static float offsetY = 0f/8f;

    private static float[] leftPos = {4f/rectangleX+offsetX, 4f/rectangleY+offsetY};//0 = col, 1 = row
    private final int rows = 640;
    private final int cols = 480;

    public boolean seesSkystone = false;
    //Como se ve en el manual del juego:
    // Pattern A = 1
    // Pattern B = 2
    // Pattern C = 3
    public int pattern = 0;

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();

    //definimos el pattern a una variable basandonos en que hay tres posibilidades, como ya se explico arriba
    public void seesStone(){
        if(valLeft == 0){
            seesSkystone =  true;
        }else{
            seesSkystone = false;
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //blanco y negro
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object


        //get values from frame
        double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeft = (int)pixLeft[0];
        seesStone();

        //create three points
        Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
        //draw circles on those points
        Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        //draw 3 rectangles
        Imgproc.rectangle(//1-3
                all,
                new Point(
                        input.cols()*(leftPos[0]-rectWidth/2),
                        input.rows()*(leftPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(leftPos[0]+rectWidth/2),
                        input.rows()*(leftPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);

        switch (stageToRenderToViewport)
        {
            case THRESHOLD:
            {
                return thresholdMat;
            }

            case detection:
            {
                return all;
            }

            case RAW_IMAGE:
            {
                return input;
            }

            default:
            {
                return input;
            }
        }
    }

}
