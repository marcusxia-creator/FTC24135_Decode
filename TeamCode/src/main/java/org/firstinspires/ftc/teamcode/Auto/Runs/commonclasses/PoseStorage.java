package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
* Simple static field serving as a storage medium for the bot's pose
* Allows different classes/opmodes to set and read from a central source of truth
* A static field allows data to persist between opmodes
 */

public class PoseStorage {
    public static Pose2D currentPose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);
    public static Pose2D endPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0);
    public static int heading = 0;
    public static int motifGreenPos;
}
