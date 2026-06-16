package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

public class Situation {
    Acceleration acceleration;
    Velocity velocity;
    Position position;

    public Situation(Acceleration acceleration, Velocity velocity, Position position){
        this.acceleration=acceleration;
        this.velocity=velocity;
        this.position=position;
    }

    public Acceleration getAcceleration() {
        return acceleration;
    }

    public Velocity getVelocity() {
        return velocity;
    }

    public Position getPosition() {
        return position;
    }
}