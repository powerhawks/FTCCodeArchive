package org.firstinspires.ftc.teamcode.HelperClasses;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Name: Transform class
 * Authors: Lincoln Doney and Chase Galey
 * Team: FTC Talons 3796 2018-2019 season
 * Date: ?
 *
 * This is for the (UNUSED) VuforiaAuto class. This was an attempt at controlling the autonomous which
 * failed and was scrapped. The class is used for the position of the robot, which theoretically was
 * supposed to drive the robot. This class essentially works like a struct but we made it a class for
 * simplicity.
 */
public class Transform
{
    //Transform Vector
    public VectorF trans;
    //Rotation Vector
    public Orientation rot;

    //Singular components for position and rotation
    public float posX;
    public float posY;
    public float posZ;
    public float rotX;
    public float rotY;
    public float rotZ;

    //Constructor for class
    public Transform(VectorF trans, Orientation rot)
    {
        this.trans = trans;
        this.rot = rot;
    }
}