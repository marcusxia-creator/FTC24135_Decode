package org.firstinspires.ftc.teamcode.IceWaddler.tests.Paths;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddlerConfig.stopTolerance;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler.Action;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler.Action.HOLDTYPE;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ExamplePath {
    public static Pose2D init = new Pose2D(METER, 0, 0, AngleUnit.DEGREES,-90);
    public static double decelX = 0.8;
    public static double Ball1X = 1;
    public static double Ball2X = 1.1;
    public static double Ball3X = 1.2;
    public static double intakeTime = 0.2; //time delay until spindexer, replace with action
    public static double shootTime = 1; //time delay for shoot, replace with action
    public static double row1Y = 0.7;
    public static Pose2D shootingPose = new Pose2D(METER, 0, 1, AngleUnit.DEGREES,-45);
    public static double fastSpeed = 0.6;
    public static double slowSpeed = 0.3;

    //Example shooting run
    public static List<Action> ShootPath = new ArrayList<>(Arrays.asList(
            new Action(init,                                                                new Pose2D(METER, decelX, row1Y, AngleUnit.DEGREES, 180),   fastSpeed,  false,  0.1, new String[]{}),
            new Action(new Pose2D(METER, decelX, row1Y, AngleUnit.DEGREES, -160),    new Pose2D(METER, Ball1X, row1Y, AngleUnit.DEGREES, 180),   slowSpeed,  true,   stopTolerance,new String[]{}),
            new Action(HOLDTYPE.POS, intakeTime, new String[]{"Intake"}),
            new Action(new Pose2D(METER, Ball1X, row1Y, AngleUnit.DEGREES, 180),    new Pose2D(METER, Ball2X, row1Y, AngleUnit.DEGREES, 180),   slowSpeed,  true,   stopTolerance,new String[]{}),
            new Action(HOLDTYPE.POS, intakeTime, new String[]{"Intake"}),
            new Action(new Pose2D(METER, Ball2X, row1Y, AngleUnit.DEGREES, 180),    new Pose2D(METER, Ball3X, row1Y, AngleUnit.DEGREES, 180),   slowSpeed,  true,   stopTolerance,new String[]{}),
            new Action(HOLDTYPE.POS, intakeTime, new String[]{"Intake"}),
            new Action(new Pose2D(METER, Ball3X, row1Y, AngleUnit.DEGREES, 180),    new Pose2D(METER, decelX, row1Y, AngleUnit.DEGREES, 180),   fastSpeed,  false,  0.1,new String[]{}),
            new Action(new Pose2D(METER, decelX, row1Y, AngleUnit.DEGREES, 180),    shootingPose,                                                       fastSpeed,  true,  stopTolerance,new String[]{}),
            new Action(HOLDTYPE.POS, shootTime, new String[]{"Shooting"}),
            new Action(shootingPose,                                                        init,                                                               fastSpeed,  true,   stopTolerance,new String[]{})
    ));

    public static List<Action> basicPath = new ArrayList<>(Arrays.asList(
            new Action(init, shootingPose, fastSpeed, true, stopTolerance, new String[]{}),
            new Action(HOLDTYPE.POS, shootTime, new String[]{}),
            new Action(shootingPose, init, fastSpeed, true, stopTolerance, new String[]{})
    ));
}
