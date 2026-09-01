package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import static org.apache.commons.math3.util.FastMath.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.velocity;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.DimMismatch;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.MotionProfile;

public class maxSpeedMP implements MotionProfile {
    Scalar maxSpeed;
    Scalar startSpeed;
    Scalar endSpeed;
    Scalar totalDist;

    public maxSpeedMP(Scalar maxSpeed) {
        if (!maxSpeed.getDimensions().equals(velocity)) {throw new DimMismatch(maxSpeed.getDimensions(), "max speed");}
        this.maxSpeed=maxSpeed;
    }

    public maxSpeedMP(){
        maxSpeed=IWConfig.maxSpeed;
    }

    @Override
    public void init(Scalar startVel, Scalar endVel, Scalar totalDistance) {
        startSpeed=startVel;
        endSpeed=endVel;
        totalDist=totalDistance;
    }

    @Override
    public Scalar getVel(double completion) {
        return new Scalar(Range.clip(
                min(startSpeed.pow(2).add(IWConfig.defaultAccel.multiply(totalDist.multiply(completion)).multiply(2)).pow(0.5).getValueSI(),
                    endSpeed.pow(2).add(IWConfig.defaultAccel.multiply(totalDist.multiply(1-completion)).multiply(2)).pow(0.5).getValueSI()),
                IWConfig.minSpeed.getValueSI(),maxSpeed.getValueSI())
                ,velocity.SIBaseUnit());
    }
}
