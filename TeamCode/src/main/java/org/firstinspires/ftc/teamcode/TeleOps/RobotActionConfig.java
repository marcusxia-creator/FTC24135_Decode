package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class RobotActionConfig {
    public static double gateDown                       = 0.13;
    public static double gateUp                         = 0.39;
    public static double DEBOUNCE_THRESHOLD             = 0.25;
    public static double spindexerSlot0                 = 0.02;
    public static double spindexerSlot1                 = 0.52;
    public static double spindexerSlot2                 = 0.96;
    public static double spindexerReset                 = 0.0;
    public static double rampDownPos                    = 0.51; //0.40 12Kg servo, 0.385 for axon micro
    public static double rampUpPos                      = 0.28; //0.18 12Kg servo, 0.215 for axon micro
    public static double angleResetPos                  = 0.0;
    //Speed
    public static double intakeSpeed                    = 0.6;
    public static double intakeStop                     = 0;
    public static double ejectSpeed                     = -0.2;
    public static double shooterVel                     = 1900;
    public static double shooterPower                   = 0.78;
    public static double shooterFactorThreshold         = 0.95;
    public static double powerFactor                    = 1;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;

    public static double getDistanceThreshold           =0.02;

    //Intake capture timing
    public static double gateDownTime                   = 0.15;
    public static double SpindexerStartTime             = 0.7;
    public static double SpindexerMoveTime              = 0.5;

    public static double a                              = 0.0001320186;
    public static double b                              = -0.01961813;
    public static double c                              = 1.431126;
    public static double correction                     = 0.053;

    //Colour Profiles
    public static double distanceThreshold              = 85;
    public static int[] none                            = {105, 110};

    public static int[] greenRangeLow                   = {120, 130};
    public static int[] greenRangeHigh                  = {135, 160};

    public static int[] purpleRangeLow                  = {115, 118};
    public static int[] purpleRangeHigh                 = {170, 230};

    public static double BALL_PRESENT_THRESHOLD_MM      = 50;
    public static final double INTAKE_TICKS_PER_REV = 145.1;
    public static final double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    //Motif IDs
    public static int GPPid                             = 21;
    public static int PGPid                             = 22;
    public static int PPGid                             = 23;

    public static final Pose2D blueAllianceResetPose    = new Pose2D(DistanceUnit.INCH, 39, 42, AngleUnit.DEGREES, 0);
    public static final Pose2D redAllianceResetPose     = new Pose2D(DistanceUnit.INCH, 39, -42, AngleUnit.DEGREES, 0);

    public static final double farEdge                  = 100.0;
    public static final double far                      = 80.0;
    public static final double mid                      = 70.0;
    public static final double close                    = 55.0;
    public static final double closeEdge                = 50.0;

    public static final double farPower                 = 0.78;
    public static final double midPower                 = 0.73;
    public static final double closePower               = 0.75;
    public static final double OZPower                  = 0.5;
}
