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
    public static double spindexerKickerInDelta         = 0.02;
    public static double spindexerZeroPos               = 0.00;
    public static double spindexerSlot1                 = 0.12;
    public static double spindexerSlot2                 = 0.31;
    public static double spindexerSlot3                 = 0.49;
    public static double spindexerSlot4                 = 0.68;
    public static double spindexerSlot5                 = 0.86;
    public static double spindexerFullPos               = 1.0;
    public static double spinderxerShootPos             = 0.08;
    public static double[] spindexerPositions           = {spindexerSlot1, spindexerSlot2, spindexerSlot3, spindexerSlot4, spindexerSlot5, spindexerFullPos};

    public static double angleResetPos                  = 0.0;
    public static double slotAngleDelta                 = 0.19;

    public static double servoStepSize                  = 0.05;     // per update() call (try 0.002–0.01)
    public static double servoTolerance                 = 0.025;    // "close enough" to finish

    public static double spindexerServoFullTime             = 0.65;    // full circle time
    public static double spindexerServoPerSlotTime          = 0.15;    // per slot time

    //kicker
    public static double kickerRetract                  = 0.37;  /// value - retract back for spindexer reversing
    public static double kickerExtend                   = 0.68; /// value - into the spindexer for pushing the ball up
    //intake Speed
    public static double intakeSpeed                    = 0.75;
    public static double intakeStop                     = 0;
    public static double ejectSpeed                     = -0.7;
    //shooter timer
    public static final long FEED_PERIOD_MS             = 350; // 0.6s per feed (tune)
    public static final double SPOOLUP_SEC              = 1.25;
    //shooter adjustor
    public static double shooterAdjusterMax             = 0.49;
    public static double shooterAdjusterMin             = 0.06;
    public static double shooterAdjusterMid             = 0.4;
    public static double shooterFactorThreshold         = 0.95;
    //drive train power
    public static double powerFactor                    = 1;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;

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

    public static double BALL_PRESENT_THRESHOLD_MM      = 50;
    public static final double INTAKE_TICKS_PER_REV     = 145.1;
    public static final double INTAKE_RPM_CONVERSION    = 60.0 / INTAKE_TICKS_PER_REV;
    public static final double SHOOTER_RPM_CONVERSION   = (60.0 / 28.0);


    //Motif IDs
    public static int GPPid                             = 21;
    public static int PGPid                             = 22;
    public static int PPGid                             = 23;

    ///public static final Pose2D blueAllianceResetPose    = new Pose2D(DistanceUnit.INCH, -64, 8, AngleUnit.DEGREES, 0);
    ///public static final Pose2D redAllianceResetPose     = new Pose2D(DistanceUnit.INCH, -64, 8, AngleUnit.DEGREES, 0);

    public static final Pose2D blueAllianceResetPose    = new Pose2D(DistanceUnit.INCH, 9, -48.5, AngleUnit.DEGREES, 0);
    public static final Pose2D redAllianceResetPose     = new Pose2D(DistanceUnit.INCH, 9, 48.5, AngleUnit.DEGREES, 0);

    public static final Pose2D redCloseGoalPose         = new Pose2D(DistanceUnit.INCH, -64, 66, AngleUnit.DEGREES, 0);
    public static final Pose2D redFarGoalPose           = new Pose2D(DistanceUnit.INCH, -66, 66, AngleUnit.DEGREES, 0);

    public static final Pose2D blueCloseGoalPose        = new Pose2D(DistanceUnit.INCH, -64, -66, AngleUnit.DEGREES, 0);
    public static final Pose2D blueFarGoalPose          = new Pose2D(DistanceUnit.INCH, -66, -66, AngleUnit.DEGREES, 0);

    public static double FAR_EDGE                 = 105.0;
    public static double FAR                      = 92.0;
    public static double MidPoint                 = 82.0;
    public static double MID                      = 72.0;
    public static double CLOSE                    = 57.0;
    public static double closeEdge                = 36.0;

    public static double FAR_ZONE_LOW             = 120.0;
    public static double FAR_ZONE_HIGH            = 167.0;

    public static int RPM0            = 3900;
    public static int RPM1            = 3350;
    public static int RPM2            = 3500;
    public static int RPM3            = 3650;
    public static int RPM4            = 3800;
    public static int RPM5            = 3900;
    public static int RPM6            = 4400;

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
}
