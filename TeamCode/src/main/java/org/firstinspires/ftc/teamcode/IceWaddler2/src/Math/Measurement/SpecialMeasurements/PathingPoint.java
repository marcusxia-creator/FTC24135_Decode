package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.velocity;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

/// A measurement used to store the position and velocity of the robot while following a path. The velocity is assumed to be perfectly tangental to the path
public class PathingPoint {
    Position position;
    Scalar velocity;


    public PathingPoint(Position position, Scalar velocity){
        this.position=position;
        if(!velocity.getDimensions().equals(velocity)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a scalar with dimensions %s and SI base units %s as velocity", velocity.getDimensions().toString(), velocity.getDimensions().SIBaseUnitStr()));}
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
