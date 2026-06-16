package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.angVelocity;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.velocity;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.time;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;

public class Velocity {
    Vector linVel;
    Scalar angVel;

    /// Creates a velocity object, thot combines a linear and angular velocity
    /// @throws RuntimeException unitError if linVel vector does not have dimensions of velocity
    /// @throws RuntimeException unitError if angVel vector does not have dimensions of angular velocity
    public Velocity(Vector linVel, Scalar angVel){
        if(!linVel.getDimensions().equals(velocity)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as a linear velocity", linVel.getDimensions().toString(), linVel.getDimensions().SIBaseUnitStr()));}
        if(!angVel.getDimensions().equals(angVelocity)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as an angular velocity", angVel.getDimensions().toString(), angVel.getDimensions().SIBaseUnitStr()));}
        this.linVel=linVel;
        this.angVel=angVel;
    }

    public Vector getLinVel(){
        return linVel;
    }

    public Scalar getAngVel(){
        return angVel;
    }

    public Velocity add(Velocity velocity){
        return new Velocity(linVel.add(velocity.getLinVel()),angVel.add(velocity.getAngVel()));
    }

    public Velocity sub(Velocity velocity){
        return new Velocity(linVel.sub(velocity.getLinVel()),angVel.sub(velocity.getAngVel()));
    }

    public Velocity multiply(double factor){
        return new Velocity(linVel.multiply(factor),angVel.multiply(factor));
    }

    public Velocity div(double factor){
        return new Velocity(linVel.div(factor),angVel.div(factor));
    }

    public Scalar mag(){
        return linVel.mag();
    }

    /// for a small change in velocity d**v** and small change in time dt, usually a tick, returns d**v**/dt, or acceleration.
    /// @throws RuntimeException unitError if dt is not in the dimension of time
    public Acceleration differentiate(Scalar dt){
        if(!dt.getDimensions().equals(time)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as the derivative of time", dt.getDimensions().toString(), dt.getDimensions().SIBaseUnitStr()));}
        return new Acceleration(linVel.div(dt),angVel.div(dt));
    }

    /// for a small change in time dt, usually a tick, returns the change in position
    /// @throws RuntimeException unitError if dt is not in the dimension of time
    public Position Integrate(Scalar dt){
        if(!dt.getDimensions().equals(time)){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot accept a vector with dimensions %s and SI base units %s as the derivative of time", dt.getDimensions().toString(), dt.getDimensions().SIBaseUnitStr()));}
        return new Position(linVel.multiply(dt),new NormalizedAngle(angVel.multiply(dt)));
    }
}