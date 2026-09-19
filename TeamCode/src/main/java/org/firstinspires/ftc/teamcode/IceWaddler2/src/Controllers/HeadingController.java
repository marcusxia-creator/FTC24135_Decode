package org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public interface HeadingController {
    Scalar getCorrection(NormalizedAngle error);
}
