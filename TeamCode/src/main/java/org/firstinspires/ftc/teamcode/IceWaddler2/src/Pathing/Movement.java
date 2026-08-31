package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

public interface Movement {

    void init(PathingPoint lastTargetPoint);

    PathingPoint getTargetPoint();

    ///All heavy calculations should happen or int getTargetVel here to save computation power
    void loop(Situation currentSituation, Scalar tickTime);

    Velocity getTargetVel();

    double getCompletion();

    Scalar getDistanceTravelled();

    String[] getTags();

    boolean finished();
}