package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.MotionProfile;

public class linearMP implements MotionProfile {
    Scalar startSpeed;
    Scalar endSpeed;
    Scalar deltaSpeed;

    public linearMP(){}

    @Override
    public void init(Scalar startVel, Scalar endVel, Scalar totalDistance) {
        startSpeed=startVel;
        endSpeed=endVel;
        deltaSpeed=endSpeed.sub(startSpeed);
    }

    @Override
    public Scalar getVel(double completion) {
        return startSpeed.add(deltaSpeed.multiply(completion));
    }
}
