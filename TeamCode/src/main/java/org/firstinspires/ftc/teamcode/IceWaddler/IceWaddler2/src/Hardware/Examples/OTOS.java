package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.m;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecondSquared;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.rad;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecondSquared;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Situation;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;

///An implementation of the SparkFun optical tracking odometry sensor (OTOS), for use with IceWaddler
public class OTOS implements IWLocalizer {
    SparkFunOTOS odo;
    public OTOS(Position offset){
        odo.setLinearUnit(METER);
        odo.setAngularUnit(RADIANS);

        odo.setLinearScalar(1.0);
        odo.setAngularScalar(1.0);

        odo.setOffset(new SparkFunOTOS.Pose2D(offset.getX().getValueSI(), offset.getY().getValueSI(), offset.getHeading().getValueSI()));
    }

    public void init(){
        odo.calibrateImu();
        odo.resetTracking();
        odo.begin();
    }

    public void reset(Situation situation) {
        odo.setPosition(new SparkFunOTOS.Pose2D(situation.getPosition().getX().getValueSI(), situation.getPosition().getY().getValueSI(), situation.getPosition().getHeading().getValueSI()));
    }

    public void update() {}

    public Situation getSituation() {
        SparkFunOTOS.Pose2D position = odo.getPosition();
        SparkFunOTOS.Pose2D velocity = odo.getVelocity();
        SparkFunOTOS.Pose2D acceleration = odo.getAcceleration();
        return new Situation(
                new Acceleration(new Vector(acceleration.x, acceleration.y, metersPerSecondSquared), new Scalar(acceleration.h, radiansPerSecondSquared)),
                new Velocity(new Vector(velocity.x, velocity.y, metersPerSecond), new Scalar(velocity.h, radiansPerSecond)),
                new Position(new Vector(position.x, position.y, m), new NormalizedAngle(position.h, rad))
        );
    }
}
