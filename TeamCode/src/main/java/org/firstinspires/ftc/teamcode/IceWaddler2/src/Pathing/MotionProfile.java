package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;

public interface MotionProfile {
    void init(Scalar startVel, Scalar endVel, Scalar totalDistance);

    Scalar getVel(double completion);
}