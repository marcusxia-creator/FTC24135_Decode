package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Situation;

public interface IWLocalizer {
    /// IceWaddler will run this once, if needed
    void init();

    /// Should reset all tracking to the given situation parameter. Reset velocity too if using accelerometer odometry
    void reset(Situation situation);

    /// Update odo. This will only be run once per tick
    void update();

    /// Should return current position, velocity, and acceleration as a situation class<br>
    /// Might be run multiple times per tick
    Situation getSituation();
}
