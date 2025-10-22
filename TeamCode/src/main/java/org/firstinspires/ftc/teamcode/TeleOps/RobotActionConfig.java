package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    public static double gateDown                       = 0.0;
    public static double gateUp                         = 1.0;
    public static double DEBOUNCE_THRESHOLD             = 0.25;
    public static double spindexerIncrement             = 0.333;
    public static double spindexerSlot1                 = 0.0;
    public static double spindexerSlot2                 = 0.5;
    public static double rampResetPos                   = 0.47; //change to 0.45
    public static double rampUpPos                      = 0.35;
    public static double angleResetPos                  = 0.0;
    //Speed
    public static double intakeSpeed                    = 1.0;

    public static double intakeRPM_THRESHOLD            = 100.0;
    public static double shooterSpeed                   = 0.9;
    public static double powerFactor                    = 0.9;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;


}

