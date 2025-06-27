package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Point;

public class Sample {

    public Point ViscenterPoint;
    public Point relPos;
    public ColorBlobLocatorProcessor.Blob blob;
    public double ODistance;

    public Sample(ColorBlobLocatorProcessor.Blob blob, Pose3D relCam, CamFieldProfile CamProfile){
        this.ViscenterPoint=CamProfile.PixelToAngle(blob.getBoxFit().center);

        double a=Math.toRadians(relCam.getOrientation().getPitch());
        double h=relCam.getPosition().z;

        double x=Math.toRadians(-ViscenterPoint.x);
        double y=Math.toRadians(-ViscenterPoint.y);

        this.relPos=new Point(
                h*(-(Math.tan(x)*Math.cos(a)/Math.tan(a-y))-(Math.tan(x)*Math.sin(a))),
                h*(-1/Math.tan(a-y))-relCam.getPosition().y);

        this.ODistance=Math.sqrt(Math.pow(this.relPos.x,2)+Math.pow(this.relPos.y,2));

        this.blob = blob;
    }
}