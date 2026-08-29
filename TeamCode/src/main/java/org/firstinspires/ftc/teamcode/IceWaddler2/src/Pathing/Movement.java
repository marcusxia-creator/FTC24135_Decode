package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

public interface Movement {

    void init(PathingPoint lastTargetPoint);

    PathingPoint getTargetPoint();

    void loop(Situation currentSituation);

    Velocity getTargetVel();

    double getCompletion();

    Scalar getDistanceTravelled();

    String[] getTags();

    boolean finished();
}