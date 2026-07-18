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

    //New spindexer values untested
    public static double spindexerzeropos   = 0.0;
    public static double spindexerSlot0     = 0.0;
    public static double spindexerSlot1     = 0.17;
    public static double spindexerSlot2     = 0.345;
    public static double spindexerSlot3     = 0.51;
    public static double spindexerSlot4     = 0.675;
    public static double spindexerSlot5     = 0.83899857;
    public static double spindexerSlot6     = 1.0;  //六七
    public static double[] spindexerSlots   = {spindexerSlot0, spindexerSlot2, spindexerSlot3, spindexerSlot4, spindexerSlot5, spindexerSlot6};

    //Special Spindexer Positions
    public static double spindexerStowPos               = (spindexerSlot0+spindexerSlot1)/2; //Position between slots 1 and 2 to prevent balls from falling out under acceleration
    public static double spindexerIntakePos             = spindexerSlot1;
    public static double spindexerShootStartPos         = 0.0;
    public static double spindexerClearanceOffset       = 0.0;     //distance between tha shoot pos and the earlier cleared slot, used in stopping. Might switch to a list of cleared positions

    public static double angleResetPos                  = 0.0;
    public static double slotAngleDelta                 = 0.19;

    public static double servoStepSize                  = 0.05;     // per update() call (try 0.002–0.01)
    public static double servoTolerance                 = 0.025;    // "close enough" to finish

    //kicker
    public static double kickerRetract                  = 0.57; /// value - retract back for spindexer reversing
    public static double kickerExtend                   = 0.82; /// value - into the spindexer for pushing the ball up
    public static double kickerRetractDelay             = 0.3;
    //intake Speed
    public static double intakeSpeed                    = 0.9;
    public static double intakeStop                     = 0;
    public static double ejectSpeed                     = -0.7;
    //shooter timer
    public static final long FEED_PERIOD_MS_CLOSE       = 200; // 250
    public static final long FEED_PERIOD_MS_FAR         = 300; // 400 250

    public static final double SPOOLUP_SEC              = 1.25;
    //shooter adjustor
    public static double shooterAdjusterMax             = 0.48;
    public static double shooterAdjusterMin             = 0.14; //new 0.14 ; old : 0.06
    public static double shooterAdjusterMid             = 0.35; //new 0.35
    public static double shooterFactorThreshold         = 0.95;
    //drive train power
    public static double powerFactor                    = 1;
    public static double accel_Slowness                 = 0.5;
    public static double decel_Slowness                 = 0.67;

    public static double getDistanceThreshold           =0.02;


    public static double a                              = 0.0001320186;
    public static double b                              = -0.01961813;
    public static double c                              = 1.431126;
    public static double correction                     = 0.053;

    //Colour Profiles
    public static int[] greenRangeLow                   = {120, 130};
    public static int[] greenRangeHigh                  = {135, 160};

    public static int[] purpleRangeLow                  = {115, 118};
    public static int[] purpleRangeHigh                 = {170, 230};

    public static int requiredSensors                   = 3;

    public static double BALL_PRESENT_THRESHOLD_MM      = 50;
    public static final double INTAKE_TICKS_PER_REV     = 145.1;
    public static final double INTAKE_RPM_CONVERSION    = 60.0 / INTAKE_TICKS_PER_REV;
    public static final double SHOOTER_RPM_CONVERSION   = (60.0 / 28.0);


    //Motif IDs
    public static int GPPid                             = 21;
    public static int PGPid                             = 22;
    public static int PPGid                             = 23;

    public static final Pose2D blueAllianceResetPose    = new Pose2D(DistanceUnit.INCH, -64, 8, AngleUnit.DEGREES, 0);
    public static final Pose2D redAllianceResetPose     = new Pose2D(DistanceUnit.INCH, -64, 8, AngleUnit.DEGREES, 0);

    ///public static final Pose2D blueAllianceResetPose    = new Pose2D(DistanceUnit.INCH, 9, -48.5, AngleUnit.DEGREES, 0);
    ///public static final Pose2D redAllianceResetPose     = new Pose2D(DistanceUnit.INCH, 9, 48.5, AngleUnit.DEGREES, 0);

    //-64 66 works
            //-63 66 previous
    public static final Pose2D redCloseGoalPose         = new Pose2D(DistanceUnit.INCH, -60, 64, AngleUnit.DEGREES, 0);
    public static final Pose2D redFarGoalPose           = new Pose2D(DistanceUnit.INCH, -66, 62, AngleUnit.DEGREES, 0);

    public static final Pose2D blueCloseGoalPose        = new Pose2D(DistanceUnit.INCH, -60, -64, AngleUnit.DEGREES, 0);
    public static final Pose2D blueFarGoalPose          = new Pose2D(DistanceUnit.INCH, -66, -62, AngleUnit.DEGREES, 0);

    public static double FAR_EDGE                 = 105.0;
    public static double FAR                      = 92.0;
    public static double MidPoint                 = 82.0;
    public static double MID                      = 68.0; ///72.0
    public static double CLOSE                    = 57.0;
    public static double closeEdge                = 36.0;

    public static double FAR_ZONE_CLOSE             = 120.0;
    public static double FAR_ZONE_MID               = 144;
    public static double FAR_ZONE_FAR               = 167.0;

    public static double RPMFactor    = 1.1;
    public static int RPM0            = 3700; ///all minus 200 rpm with upgraded hardware
    public static int RPM1            = 3150;
    public static int RPM2            = 3150;
    public static int RPM3            = 3450;
    public static int RPM4            = 3600;
    public static int RPM5            = 3700;
    public static int RPM6            = 4500;
    public static int RPM7            = 4800;

    public static final double farPower                 = 0.78;     // 3920 - 0.8  @ 12.73
    public static final double midPower                 = 0.73;     // 3360 - 0.70 @ 12.82
    public static final double closePower               = 0.75;     // 3700 - 0.75 @ 13.00
    public static final double OZPower                  = 0.5;
    public static final double FZPower                  = 0.93;

    public static final double turret_Center_X_Offset   = 0.0127;
    public static final double turret_Center_Y_Offset   = 0.028575;

    public static final int shooterMaxRPM               = 5200; //at 13.1 volts, power : 1
    public static final double REF_VOLTAGE              = 13.1;

    public static final double aimingAngleThrehold      = 5;
    public static final double turretCameraRadius       = 0.1778;// for limelight aiming angle correction

    public static final double trimStep                 = 10;
    public static final double adjSpeed                 = 1500;
    public static final double pressTime                = 0.2;
}
