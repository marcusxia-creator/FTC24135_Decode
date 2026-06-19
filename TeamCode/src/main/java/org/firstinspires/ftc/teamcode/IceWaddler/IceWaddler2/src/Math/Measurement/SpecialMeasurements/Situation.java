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

    public void setAcceleration(Acceleration acceleration) {
        this.acceleration=acceleration;
    }

    public void setVelocity(Velocity velocity) {
        this.velocity=velocity;
    }

    public void setPosition(Position position) {
        this.position=position;
    }
}