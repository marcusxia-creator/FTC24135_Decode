package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

public class proportionaHeadingController implements HeadingController {
    Scalar kP;
    public proportionaHeadingController(Scalar kP){
        this.kP=kP;
    }

    @Override
    public Scalar getCorrection(NormalizedAngle error) {
        return error.toScalar().multiply(kP);
    }
}