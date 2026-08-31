package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;

public class linearHeadingProfile implements HeadingProfile {
    NormalizedAngle startHeading;
    NormalizedAngle endHeading;
    NormalizedAngle deltaHeading;

    public linearHeadingProfile(){}

    @Override
    public void init(NormalizedAngle startAngle, NormalizedAngle endAngle, Scalar totalDistance) {
        startHeading=startAngle;
        endHeading=endAngle;
        deltaHeading=endHeading.sub(startHeading);
    }

    @Override
    public NormalizedAngle getAng(double completion) {
        return startHeading.add(deltaHeading.multiply(completion));
    }
}
