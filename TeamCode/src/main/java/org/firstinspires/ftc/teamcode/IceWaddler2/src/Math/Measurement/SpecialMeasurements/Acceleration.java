package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.acceleration;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.angAcceleration;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.time;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.velocity;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.DimlessVector;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Vector;

public class Acceleration {
    Vector linAcc;
    Scalar angAcc;

    /// Creates an Acceleration object, thot combines a linear and angular acceleration
    /// @throws RuntimeException unitError if linAcc vector does not have dimensions of acceleration
    /// @throws RuntimeException unitError if angAcc vector does not have dimensions of angular acceleration
    public Acceleration(Vector linAcc, Scalar angAcc){
        if(!linAcc.getDimensions().equals(acceleration)){throw new DimMismatch(linAcc.getDimensions(),"linear acceleration");}
        if(!angAcc.getDimensions().equals(angAcceleration)){throw new DimMismatch(linAcc.getDimensions(),"angular acceleration");}
        this.linAcc=linAcc;
        this.angAcc=angAcc;
    }

    public Vector getLinAcc(){
        return linAcc;
    }

    public Scalar getAngAcc(){
        return angAcc;
    }

    /// Rotates the vector by normalized angle {@code angle} clockwise
    /// @param angle the angle to rotate by
    /// @return the rotated vector
    /// note: does not rotate angular acceleration
    public Acceleration rotateBy(NormalizedAngle angle){
        return new Acceleration(linAcc.rotateBy(angle),angAcc);
    }

    public Acceleration add(Acceleration accel){
        return new Acceleration(linAcc.add(accel.getLinAcc()),angAcc.add(accel.getAngAcc()));
    }

    public Acceleration sub(Acceleration accel){
        return new Acceleration(linAcc.sub(accel.getLinAcc()),angAcc.sub(accel.getAngAcc()));
    }

    public Acceleration multiply(double factor){
        return new Acceleration(linAcc.multiply(factor),angAcc.multiply(factor));
    }

    public Acceleration div(double factor){
        return new Acceleration(linAcc.div(factor),angAcc.div(factor));
    }

    public Scalar mag(){
        return linAcc.mag();
    }

    public DimlessVector unitVector(){
        return linAcc.unitVector();
    }

    /// for a small change in time dt, usually a tick, returns the change in velocity
    /// @throws RuntimeException unitError if dt is not in the dimension of time
    public Velocity integrate(Scalar dt){
        if(!dt.getDimensions().equals(time)){throw new DimMismatch(dt.getDimensions(),"derivative of time");}
        return new Velocity(linAcc.multiply(dt),angAcc.multiply(dt));
    }

    public Scalar getX(){
        return linAcc.getX();
    }

    public Scalar getY(){
        return linAcc.getY();
    }

    public Scalar getHeading(){
        return angAcc;
    }

    public static Acceleration zero=new Acceleration(new Vector(0,0,acceleration.SIBaseUnit()),new Scalar(0,angAcceleration.SIBaseUnit()));
}