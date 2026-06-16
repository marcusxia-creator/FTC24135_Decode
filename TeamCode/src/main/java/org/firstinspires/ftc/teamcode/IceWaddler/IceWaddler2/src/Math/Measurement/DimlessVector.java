package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.IWMath.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public class DimlessVector {
    double x;
    double y;

    ///Creates a new 2D dimensionless vector from x and y values
    public DimlessVector(double x,double y){
        this.x=x;
        this.y=y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double mag(){
        return sqrt(pow(x,2)+pow(y,2));
    }

    ///Returns a normalized angle representing the direction of the vector
    /// note: All angles are measured from the positive y-axis towards the positive x-axis (clockwise)
    public NormalizedAngle direction(){
        return arcTan2(y, x);
    }

    public DimlessVector add(DimlessVector vector){
        return new DimlessVector(x+vector.x,y+vector.y);
    }

    public DimlessVector sub(DimlessVector vector){
        return new DimlessVector(x-vector.x,y-vector.y);
    }

    public DimlessVector multi(double factor){
        return new DimlessVector(x*factor,y*factor);
    }

    public DimlessVector div(double factor){
        return new DimlessVector(x/factor,y/factor);
    }

    /// @return the unit vector with the same direction as the vector, but with magnitude of 1
    /// @throws RuntimeException "mathError" if the vector has a magnitude of 0
    public DimlessVector unitVector(){
        if(mag()==0){
            throw new RuntimeException("mathError: Cannot take the unit vector of a vector with magnitude 0");
        }
        return div(mag());
    }

    /// Rotates the vector by normalized angle {@code angle} clockwise
    /// @param angle the angle to rotate by
    /// @return the rotated vector
    public DimlessVector rotateBy(NormalizedAngle angle){
        return new DimlessVector((x*cos(angle))+(y*sin(angle)),(y*cos(angle))-(x*sin(angle)));
    }
}