package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    public static double gateDown                       = 0.12;
    public static double gateUp                         = 0.30;
    public static double DEBOUNCE_THRESHOLD             = 0.25;
    public static double spindexerSlot0                 = 0.02;
    public static double spindexerSlot1                 = 0.53;
    public static double spindexerSlot2                 = 0.95;
    public static double spindexerReset                 = 0.0;
    public static double rampDownPos                    = 0.42; //changed from 0.45
    public static double rampUpPos                      = 0.12;
    public static double angleResetPos                  = 0.0;
    //Speed
    public static double intakeSpeed                    = 0.6;
    public static double ejectSpeed                     = -0.2;
    public static double shooterVel                     = 1900;
    public static double shooterFactorThreshold         = 0.95;
    public static double powerFactor                    = 0.9;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;

    public static double getDistanceThreshold           =0.02;

    //Intake capture timing
    public static double gateDownTime                   = 0.1;
    public static double SpindexerStartTime             = 0.7;
    public static double SpindexerMoveTime              = 1.0;

    public static double a                              = 0;
    public static double exp                            = 1;

    //Colour Profiles
    public static double distanceThreshold              = 100;
    public static int[] none                            = {105, 110};

    public static int[] greenRangeLow                   = {120, 130};
    public static int[] greenRangeHigh                  = {145, 165};

    public static int[] purpleRangeLow                  = {115, 118};
    public static int[] purpleRangeHigh                 = {180, 230};

    public static double BALL_PRESENT_THRESHOLD_MM      = 50;
}