package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.mm;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.rad;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.s;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddlerConfig1.xEncoderDirection;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddlerConfig1.yEncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Situation;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;

///An implementation of the goBilda pinpoint odometry computer, for use with IceWaddler
public class goBildaOdoComputer implements IWLocalizer {
    GoBildaPinpointDriver odo;
    goBildaOdoComputer(GoBildaPinpointDriver odo, Scalar odoXOffset, Scalar odoYOffset, GoBildaPinpointDriver.GoBildaOdometryPods pods, GoBildaPinpointDriver.EncoderDirection xEncoderDirection, GoBildaPinpointDriver.EncoderDirection yEncoderDirection){
        this.odo=odo;
        odo.setOffsets(odoXOffset.getValue(mm), odoYOffset.getValue(mm));
        odo.setEncoderResolution(pods);
        odo.setEncoderDirections(xEncoderDirection, yEncoderDirection);
    }

    public void init(){
        //Set to start counting at initPose parameter
        odo.initialize();
        odo.resetPosAndIMU();
    }

    public void reset(Situation situation) {
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.METER, situation.getPosition().getX().getValueSI(), situation.getPosition().getY().getValueSI(), AngleUnit.RADIANS, situation.getPosition().getHeading().getValueSI()));
    }

    public void update() {
        odo.update();
    }

    public Situation getSituation() {
        return new Situation(
                null,
                new Velocity(new Vector(odo.getVelX(), odo.getVelY(), mm.div(s)), new Scalar(odo.getHeadingVelocity(),radiansPerSecond)),
                new Position(new Vector(odo.getPosX(), odo.getPosY(), mm), new NormalizedAngle(odo.getHeading(), rad))
        );
    }
}
