package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import static org.apache.commons.math3.util.FastMath.pow;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.minSpeed;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.MotionProfile;

public class SCurveMP implements MotionProfile {
    Scalar startSpeed;
    Scalar endSpeed;
    Scalar deltaSpeed;

    public SCurveMP(){}

    @Override
    public void init(Scalar startVel, Scalar endVel, Scalar totalDistance) {
        startSpeed=startVel.greaterThan(minSpeed)?startVel:minSpeed;
        endSpeed=endVel.greaterThan(minSpeed)?endVel:minSpeed;
        deltaSpeed=endSpeed.sub(startSpeed);
    }

    @Override
    public Scalar getVel(double completion) {
        return startSpeed.add(deltaSpeed.multiply(completion<=0.5?(2*pow(completion,2)):(1-2*pow(completion-1,2))));
    }
}
