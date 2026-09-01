package org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

import java.util.List;

public interface IWDriveTrain {
    /// This method is run once at the beginning of the program, leave blank if not needed
    /// After init, all motors should be reversed such that all motors running at power +1 yields forward acceleration
    void init();

    /// Should write each power, between 1 and -1, to the corresponding motor<br>
    /// IceWaddler currently only supports mecanum and omni wheel drives
    void runPowers(double FL_Power, double BL_Power, double FR_Power, double BR_Power);

    /// Should return the ticks/second for each motor as a list, in the order of:
    /// - Front left
    /// - Back left
    /// - Front right
    /// - Back right<br>
    /// Use {@code Arrays.asList()} when creating list
    List<Double> getVelocities();

    void runAccel(Scalar FL_Accel, Scalar BL_Accel, Scalar FR_Accel, Scalar BR_Accel);

    double powerController(Scalar Accel, double motorVel);
}