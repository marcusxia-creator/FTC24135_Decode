package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import static org.apache.commons.math3.util.FastMath.pow;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;

public class SCurveHP implements HeadingProfile {
    NormalizedAngle startHeading;
    NormalizedAngle endHeading;
    NormalizedAngle deltaHeading;
    Scalar totalDistance;

    public SCurveHP(){}

    @Override
    public void init(NormalizedAngle startAngle, NormalizedAngle endAngle, Scalar totalDistance) {
        startHeading=startAngle;
        endHeading=endAngle;
        deltaHeading=endHeading.sub(startHeading);

        this.totalDistance=totalDistance;
    }

    @Override
    public NormalizedAngle getHeading(double completion) {
        return startHeading.add(deltaHeading.multiply(completion<=0.5?(2*pow(completion,2)):(1-2*pow(completion-1,2))));
    }

    @Override
    public Scalar getAngVel(double completion, Scalar velocity) {
        return deltaHeading.multiply(completion<=0.5?4*completion:4*(1-completion)).multiply(velocity).div(totalDistance);
    }
}
