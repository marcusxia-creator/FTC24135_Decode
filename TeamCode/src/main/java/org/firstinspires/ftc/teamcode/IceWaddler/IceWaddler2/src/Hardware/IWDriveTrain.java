package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware;

import java.util.List;

public interface IWDriveTrain {
    /// This method is run once at the beginning of the program, leave blank if not needed
    /// After init, all motors should be reversed such that all motors running at power +1 yields clockwise rotation
    void init();

    /// Should write each power, between 1 and -1, to the corresponding motor<br>
    /// IceWaddler currently only supports mecanum and omni wheel drives
    public void writePowers(double FL_Power, double BL_Power, double FR_Power, double BR_Power);

    /// Should return the ticks/second for each motor as a list, in the order of:
    /// - Front left
    /// - Back left
    /// - Front right
    /// - Back right<br>
    /// Use {@code Arrays.asList()} when creating list
    public List<Double> getVelocities();
}