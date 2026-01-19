package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    public static double gateInitDown                    = 0.0;

    public static double GATEDOWN                        = 0.15;
    public static double GATEUP                          = 0.32;
    public static double DEBOUNCE_THRESHOLD              = 0.25;
    //Sorter
    public static double spindexerSlot1                  = 0.18;
    public static double spindexerSlot2                  = 0.43;
    public static double spindexerSlot3                  = 0.68;
    public static double RAMP_RESET_POSITION            = 0.56; //change from 0.58 (init)
    public static double RAMP_UP                        = 0.31; //change from 0.34 (init)

    public static double spindexerKickIn                = 0.00;
    public static double spindexerSlotShoot             = 0.25;
    public static double spindexerSlot0                 = 0.00;

    //Intake Speed
    public static double INTAKE_POWER                   = 0.6;
    public static double INTAKE_REVERSE_POWER           = -0.2;
    public static double INTAKE_HEAVYJAM_REVERSE_SPEED  = -0.8;
    public static double intakeRPM_THRESHOLD            = 100.0;
    public static double BALL_PRESENT_THRESHOLD_MM      = 55.0; // adjust per sensor mounting

    //Offtake speed
    public static final double SHOOTER_POWER    = 0.85;
    public static final double RAMP_UP_TIME_1st = 1.25;
    public static final double RAMP_UP_TIME     = 0.75;
    public static final double FIRE_TIME        = 1.25;
    public static final double EJECT_TIME       = 0.1;

    public static double kickerIn                       = 0.64;
    public static double kickerOut                      = 0.28;
    public static double shooterAdjusterMax             = 0.6;
    public static double shooterAdjusterMin             = 0.82;

    //Drive Train
    public static double POWERFACTOR            = 0.9;
    public static double ACCEL_SLOWNESS         = 0.25;
    public static double DECEL_SLOWNESS         = 0.5;

    //Distance to goal
    public static double DISTANCETOGOAL         = 0.0;

    public static final double farEdge                  = 100.0;
    public static final double far                      = 80.0;
    public static final double mid                      = 70.0;
    public static final double close                    = 55.0;
    public static final double closeEdge                = 50.0;

    public static final double FAR_ZONE_LOW             = 151.0;
    public static final double FAR_ZONE_HIGH            = 171.0;

    public static final double farPower                 = 0.78;
    public static final double midPower                 = 0.73;
    public static final double closePower               = 0.75;
    public static final double ozPower                  = 0.5;


}

