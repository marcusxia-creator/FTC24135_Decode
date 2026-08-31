package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.length;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.time;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.DimlessVector;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Vector;

public class Position {
    Vector linPos;
    NormalizedAngle angPos;

    /// Creates a Position object, thot combines a linear and angular position
    /// @throws RuntimeException unitError if linPos vector does not have spacial dimensions
    public Position(Vector linPos, NormalizedAngle angPos){
        if(!linPos.getDimensions().equals(length)){throw new DimMismatch(linPos.getDimensions(),"linear position");}
        this.linPos=linPos;
        this.angPos=angPos;
    }

    public Vector getLinPos(){
        return linPos;
    }

    public NormalizedAngle getAngPos(){
        return angPos;
    }

    public Position add(Position position){
        return new Position(linPos.add(position.getLinPos()),angPos.add(position.getAngPos()));
    }

    public Position sub(Position position){
        return new Position(linPos.sub(position.getLinPos()),angPos.sub(position.getAngPos()));
    }

    public Position multiply(double factor){
        return new Position(linPos.multiply(factor),angPos);
    }

    public Position div(double factor){
        return new Position(linPos.div(factor),angPos);
    }

    public Scalar mag(){
        return linPos.mag();
    }

    public DimlessVector unitVector(){
        return linPos.unitVector();
    }

    /// for a small change in position d**P** and small change in time dt, usually a tick, returns d**P**/dt, or velocity.
    /// @throws RuntimeException unitError if dt is not in the dimension of time
    public Velocity differentiate(Scalar dt){
        if(!dt.getDimensions().equals(time)){throw new DimMismatch(dt.getDimensions(),"derivative of time");}
        return new Velocity(linPos.div(dt),angPos.toScalar().div(dt));
    }

    public Scalar getX(){
        return linPos.getX();
    }

    public Scalar getY(){
        return linPos.getY();
    }

    public NormalizedAngle getHeading(){
        return angPos;
    }
}