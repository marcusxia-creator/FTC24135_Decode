package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import org.opencv.core.Point;

public class CamFieldProfile {

    public double FOVx;
    public double FOVy;

    public double Sizex;
    public double Sizey;


    public CamFieldProfile(double FOVx, double FOVy, double Sizex, double Sizey){
        this.FOVx=FOVx;
        this.FOVy=FOVy;

        this.Sizex=Sizex;
        this.Sizey=Sizey;
    }

    public Point PixelToAngle(Point screenPoint){
        return new Point(
                ((screenPoint.x-(Sizex/2))*FOVx)/Sizex,
                ((screenPoint.y-(Sizey/2))*FOVy)/Sizey
        );
    }
}
