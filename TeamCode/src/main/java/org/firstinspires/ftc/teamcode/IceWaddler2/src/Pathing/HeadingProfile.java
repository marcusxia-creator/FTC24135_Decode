package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public interface HeadingProfile {
    void init(NormalizedAngle startAngle, NormalizedAngle endAngle, Scalar totalDistance);

    NormalizedAngle getHeading(double completion);

    Scalar getAngVel(double completion, Scalar velocity);
}
