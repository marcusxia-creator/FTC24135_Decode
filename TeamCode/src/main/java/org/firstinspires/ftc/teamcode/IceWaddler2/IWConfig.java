package org.firstinspires.ftc.teamcode.IceWaddler2;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.LatPositionController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.PrebuiltControllers.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

@Config
public class IWConfig {
    // Ticks in derivative, to reduce noise
    public static int derivativeTicks =2;

    // Velocity -> Acceleration Error Correction
    public static double linAccelKP=7;
    public static double angAccelKP=1.3;
    public static AccelerationController accelerationController=new proportionalAccController(linAccelKP,angAccelKP);

    // Positional error -> Velocity Correction
    public static double latPosKP=7;
    public static double headingKP=12;
    public static LatPositionController latPosController= new proportionaLatController(latPosKP);
    public static HeadingController headingController= new proportionaHeadingController(headingKP);

    // General constraints
    public static Scalar maxAccel               = new Scalar(6, metersPerSecondSquared); //Maximum acceleration before wheels slip
    public static Scalar maxAngAccel            = new Scalar(1, radiansPerSecondSquared); //Maximum angular acceleration before wheels slip

    // Positional control parameter defaults, Can be modified per action
    public static Scalar maxSpeed               = new Scalar(1.4, metersPerSecond);
    public static Scalar minSpeed               = new Scalar(0.2, metersPerSecond);// A minimum drive speed, to prevent stalls
    public static Scalar defaultAccel           = new Scalar(1,metersPerSecondSquared); // A "comfortable" acceleration
    public static Scalar distThreshold          = new Scalar(1, cm); // The longitudinal distance from the end point at which the action indicates completion

    // Angular position parameter defaults
    public static Scalar angThreshold    = new Scalar(2, deg); // The angular distance from the end point at which the action indicates completion, used for non-distance driven movements
}