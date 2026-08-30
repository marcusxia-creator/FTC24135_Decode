package org.firstinspires.ftc.teamcode.IceWaddler2;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.AccelerationController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.LatPositionController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Controllers.MotorController;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;

import java.util.function.Function;

public class IWConfig {
    // Acceleration -> Motor Power Controller
    public static MotorController motorController;

    // Velocity -> Acceleration Error Correction
    public static AccelerationController accelerationController;

    // Positional error -> Velocity Correction
    public static LatPositionController latPosController;
    public static HeadingController headingController;

    // General constraints
    public static Scalar maxAccel               = new Scalar(3, metersPerSecondSquared); //Maximum acceleration before wheels slip
    public static Scalar maxAngAccel            = new Scalar(1, radiansPerSecondSquared); //Maximum angular acceleration before wheels slip
    public static Scalar wheelPivotRadius       = new Scalar(10.5, in); //The distance between the pivot point and each of the wheels, or half the length of the diagonal

    // Positional control parameter defaults, Can be modified per action
    public static Scalar defaultMaxSpeed        = new Scalar(2, metersPerSecond);
    public static Scalar defaultMinSpeed        = new Scalar(0.1, metersPerSecond);
    public static Scalar defaultAccel           = maxAccel.multiply(0.5); // A "comfortable" acceleration
    public static Scalar defaultLeadDistance    = new Scalar(0.3, m); // The distance ahead from which the robot is "led", shorter values yield stronger corrections
    public static Scalar defaultDistThreshold   = new Scalar(0.05, m); // The longitudinal distance from the end point at which the action indicates completion

    // Angular position parameter defaults
    public static Scalar defaultAngMaxSpeed     = new Scalar(2, radiansPerSecond);
    public static Scalar defaultAngMinSpeed     = new Scalar(0.1, radiansPerSecond);
    public static Scalar defaultAngAccel        = maxAngAccel.multiply(0.5); // A "comfortable" acceleration
    public static Scalar defaultAngThreshold    = new Scalar(4, deg); // The angular distance from the end point at which the action indicates completion
}
