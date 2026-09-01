package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.LatPositionController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public class proportionaLatController implements LatPositionController {
    Scalar kP;
    public proportionaLatController(Scalar kP){
        this.kP=kP;
    }

    @Override
    public Scalar getCorrection(Scalar error) {
        return error.multiply(kP);
    }
}