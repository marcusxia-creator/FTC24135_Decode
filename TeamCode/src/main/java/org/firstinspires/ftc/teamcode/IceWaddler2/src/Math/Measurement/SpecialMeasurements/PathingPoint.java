package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.velocity;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions;

/// A measurement used to store the position and velocity of the robot while following a path. The velocity is assumed to be perfectly tangental to the path
public class PathingPoint {
    Position position;
    Scalar velocity;


    public PathingPoint(Position position, Scalar velocity){
        this.position=position;
        if(!velocity.getDimensions().equals(Dimensions.velocity)){throw new DimMismatch(velocity.getDimensions(),"velocity");}
        this.velocity=velocity;
    }

    public PathingPoint(Position position){
        this(position, new Scalar(0,metersPerSecond));
    }

    public Position getPosition() {
        return position;
    }

    public Scalar getVelocity(){
        return velocity;
    }
}
