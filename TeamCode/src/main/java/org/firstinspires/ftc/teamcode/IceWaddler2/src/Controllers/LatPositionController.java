package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

public interface LatPositionController {
    Scalar getCorrection(Scalar error);
}
