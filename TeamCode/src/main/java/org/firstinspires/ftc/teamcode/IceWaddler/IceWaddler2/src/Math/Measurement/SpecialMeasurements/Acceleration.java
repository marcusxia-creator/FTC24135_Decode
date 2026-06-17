package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.acceleration;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.angAcceleration;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.time;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;

public class Acceleration {
    Vector linAcc;
    Scalar angAcc;

    /// Creates an Acceleration object, thot combines a linear and angular acceleration
    /// @throws RuntimeException unitError if linAcc vector does not have dimensions of acceleration
    /// @throws RuntimeException unitError if angAcc vector does not have dimensions of angular acceleration
    public Acceleration(Vector linAcc, Scalar angAcc){
        if(!linAcc.getDimensions().equals(acceleration)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as a linear acceleration", linAcc.getDimensions().toString(), linAcc.getDimensions().SIBaseUnitStr()));}
        if(!angAcc.getDimensions().equals(angAcceleration)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as an angular acceleration", angAcc.getDimensions().toString(), angAcc.getDimensions().SIBaseUnitStr()));}
        this.linAcc=linAcc;
        this.angAcc=angAcc;
    }

    public Vector getLinAcc(){
        return linAcc;
    }

    public Scalar getAngAcc(){
        return angAcc;
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

    /// for a small change in time dt, usually a tick, returns the change in velocity
    /// @throws RuntimeException unitError if dt is not in the dimension of time
    public Velocity integrate(Scalar dt){
        if(!dt.getDimensions().equals(time)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as the derivative of time", dt.getDimensions().toString(), dt.getDimensions().SIBaseUnitStr()));}
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
}