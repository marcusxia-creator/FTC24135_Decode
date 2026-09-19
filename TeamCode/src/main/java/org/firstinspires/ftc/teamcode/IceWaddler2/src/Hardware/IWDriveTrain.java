package org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.DimlessVector;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

import java.util.List;

public interface IWDriveTrain {
    /// This method is run once at the beginning of the program, leave blank if not needed
    /// After init, all motors should be reversed such that all motors running at power +1 yields forward acceleration
    default void init(){};

    /// Should write power to each of the motors based on your robot specific drivetrain formula.
    /// See example code.
    /// IceWaddler currently only supports mecanum and omni wheel drives
    void run(DimlessVector linPower, double angularPower);

    /// Ask drivetrain to drive at a certain acceleration while currently moving at a certain velocity
    /// Both vectors are fieldcentric
    void run(Velocity currentVelocity, Acceleration targetAcceleration, NormalizedAngle currentHeading);

    void runPower(double FL_Power, double BL_Power, double FR_Power, double BR_Power);
}