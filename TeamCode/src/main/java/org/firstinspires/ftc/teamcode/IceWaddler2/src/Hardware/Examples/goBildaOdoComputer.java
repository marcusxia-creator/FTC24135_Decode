package org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.Examples;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

///An implementation of the goBilda pinpoint odometry computer, for use with IceWaddler
public class goBildaOdoComputer implements IWLocalizer {
    GoBildaPinpointDriver odo;
    public goBildaOdoComputer(GoBildaPinpointDriver odo, Scalar odoXOffset, Scalar odoYOffset, GoBildaPinpointDriver.GoBildaOdometryPods pods, GoBildaPinpointDriver.EncoderDirection xEncoderDirection, GoBildaPinpointDriver.EncoderDirection yEncoderDirection){
        this.odo=odo;
        odo.setOffsets(odoXOffset.getValue(m), odoYOffset.getValue(m),DistanceUnit.METER);
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
        odo.setPosition(new Pose2D(DistanceUnit.METER, situation.getPosition().getY().getValueSI(), -situation.getPosition().getX().getValueSI(), AngleUnit.RADIANS, -situation.getPosition().getHeading().getValueSI()));
    }

    public void update() {
        odo.update();
    }

    public Situation getSituation() {
        return new Situation(
                null,
                new Velocity(new Vector(-odo.getVelY(DistanceUnit.METER), odo.getVelX(DistanceUnit.METER), mm.div(s)), new Scalar(-odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS),radiansPerSecond)),
                new Position(new Vector(-odo.getPosY(DistanceUnit.METER), odo.getPosX(DistanceUnit.METER), mm), new NormalizedAngle(-odo.getHeading(AngleUnit.RADIANS), rad))
        );
    }
}
