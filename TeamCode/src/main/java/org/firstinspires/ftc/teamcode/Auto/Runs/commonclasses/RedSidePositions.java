package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class RedSidePositions {
    public static double CloseShotPower = 0.69;
    public static double FarShotPower = 0.84;

    private final Pose2d redGoalPose  = new Pose2d(-70,70,-45);

    public static double IntakeSet1Position1_X = 36;
    public static double IntakeSet1Position1_Y = 24;
    public static double IntakeSet1Position4_X = 36;
    public static double IntakeSet1Position4_Y = 60;

    public static double Close_IntakeSet2Position1_X = 13;
    public static double Close_IntakeSet2Position1_Y = 32;
    public static double Close_IntakeSet2Position4_X = 13;
    public static double Close_IntakeSet2Position4_Y = 58;

    public static double IntakeSet3Position1_X = -10;
    public static double IntakeSet3Position1_Y = 40;
    public static double IntakeSet3Position4_X = -10;
    public static double IntakeSet3Position4_Y = 54;

    public static double IntakeHPPosition1_X = 58;
    public static double IntakeHPPosition1_Y = 24;
    public static double IntakeHPPosition4_X = 58;
    public static double IntakeHPPosition4_Y = 60;

    public static double IntakeHP2Position1_X = 32;
    public static double IntakeHP2Position1_Y = 24;
    public static double IntakeHP2Position4_X = 50;
    public static double IntakeHP2Position4_Y = 60;

    public static double GateIntakePosition_X = 14;
    public static double GateIntakePosition_Y = 61.5;
    public static double GateIntakePosition_Heading = 120;

    public static double FarShootingPosition_X = 52;
    public static double FarShootingPosition_Y = 16;
    public static double FarShootingPosition_Heading = 90;

    public static double CloseShootingPosition_X = -10;
    public static double CloseShootingPosition_Y = 16;
    public static double CloseShootingPosition_Heading = 90;
}