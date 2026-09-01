package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import static org.apache.commons.math3.util.FastMath.pow;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;

public class SCurveHP implements HeadingProfile {
    NormalizedAngle startHeading;
    NormalizedAngle endHeading;
    NormalizedAngle deltaHeading;

    public SCurveHP(){}

    @Override
    public void init(NormalizedAngle startAngle, NormalizedAngle endAngle, Scalar totalDistance) {
        startHeading=startAngle;
        endHeading=endAngle;
        deltaHeading=endHeading.sub(startHeading);
    }

    @Override
    public NormalizedAngle getAng(double completion) {
        return startHeading.add(deltaHeading.multiply(completion<=0.5?(2*pow(completion,2)):(1-2*pow(completion-1,2))));
    }
}
