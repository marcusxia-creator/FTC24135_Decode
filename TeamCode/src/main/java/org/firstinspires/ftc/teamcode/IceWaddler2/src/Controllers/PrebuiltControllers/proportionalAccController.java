package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

public class proportionalAccController implements AccelerationController {
    Scalar linearkP;
    Scalar angularkP;
    public proportionalAccController(Scalar linearkP, Scalar angularkP){
        this.linearkP=linearkP;
        this.angularkP=angularkP;
    }

    @Override
    public Acceleration getCorrection(Velocity error) {
        return new Acceleration(error.getLinVel().multiply(linearkP),error.getAngVel().multiply(angularkP));
    }
}
