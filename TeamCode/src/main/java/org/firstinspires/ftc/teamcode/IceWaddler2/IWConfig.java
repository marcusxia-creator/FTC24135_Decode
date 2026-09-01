package org.firstinspires.ftc.teamcode.IceWaddler2;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.LatPositionController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

public class IWConfig {
    // Velocity -> Acceleration Error Correction
    public static AccelerationController accelerationController=new proportionalAccController(new Scalar(-10,perSecond),new Scalar(-10,perSecond));

    // Positional error -> Velocity Correction
    public static LatPositionController latPosController= new proportionaLatController(new Scalar(-10,perSecond));
    public static HeadingController headingController= new proportionaHeadingController(new Scalar(-10,perSecond));

    // General constraints
    public static Scalar maxAccel               = new Scalar(8, metersPerSecondSquared); //Maximum acceleration before wheels slip
    public static Scalar maxAngAccel            = new Scalar(1, radiansPerSecondSquared); //Maximum angular acceleration before wheels slip
    public static Scalar wheelPivotRadius       = new Scalar(10.5, in); //The distance between the pivot point and each of the wheels, or half the length of the diagonal

    // Positional control parameter defaults, Can be modified per action
    public static Scalar maxSpeed               = new Scalar(2, metersPerSecond);
    public static Scalar minSpeed               = new Scalar(0.1, metersPerSecond);// A minimum drive speed, to prevent stalls
    public static Scalar defaultAccel           = maxAccel.multiply(0.5); // A "comfortable" acceleration
    public static Scalar distThreshold          = new Scalar(0.05, m); // The longitudinal distance from the end point at which the action indicates completion

    // Angular position parameter defaults
    public static Scalar angThreshold    = new Scalar(4, deg); // The angular distance from the end point at which the action indicates completion, used for non-distance driven movements
}