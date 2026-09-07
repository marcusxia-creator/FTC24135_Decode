package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.latPosKP;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.perSecond;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.LatPositionController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public class proportionaLatController implements LatPositionController {
    public proportionaLatController(double kP){

    }

    @Override
    public Scalar getCorrection(Scalar error) {
        return error.multiply(new Scalar(-latPosKP,perSecond));
    }
}