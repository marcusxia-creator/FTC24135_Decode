package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    public static double gateInitDown                    = 0.0;

    public static double GATEDOWN                        = 0.125;
    public static double GATEUP                          = GATEDOWN+0.18;
    public static double DEBOUNCE_THRESHOLD              = 0.25;
    public static double spindexerIncrement              = 0.333;
    //Sorter
    public static double spindexerSlot1                  = 0.02;
    public static double spindexerSlot2                  = 0.46;
    public static double spindexerSlot3                  = 0.90;
    public static double RAMP_RESET_POSITION            = 0.42; //change to 0.45
    public static double RAMP_UP                        = 0.12;
    public static double angleResetPos                  = 0.0;
    //Intake Speed
    public static double INTAKE_POWER                   = 0.6;
    public static double INTAKE_REVERSE_POWER           = -0.2;
    public static double INTAKE_HEAVYJAM_REVERSE_SPEED  = -0.8;
    public static double intakeRPM_THRESHOLD            = 100.0;

    //Offtake speed
    public static final double SHOOTER_POWER    = 0.85;
    public static final double RAMP_UP_TIME     = 0.75;
    public static final double FIRE_TIME        = 1.25;
    public static final double EJECT_TIME       = 0.25;

    //Drive Train
    public static double powerFactor                    = 0.9;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;


}

