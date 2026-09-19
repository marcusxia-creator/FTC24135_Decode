package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.perSecond;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

public class proportionalAccController implements AccelerationController {
    public proportionalAccController(double linearkP, double angularkP){}

    @Override
    public Acceleration getCorrection(Velocity error) {
        return new Acceleration(error.getLinVel().multiply(new Scalar(-linAccelKP,perSecond)),error.getAngVel().multiply(new Scalar(-angAccelKP,perSecond)));
    }
}