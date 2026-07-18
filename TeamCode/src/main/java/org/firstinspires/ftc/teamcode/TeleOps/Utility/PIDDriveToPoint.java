package org.firstinspires.ftc.teamcode.TeleOps.Utility;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class PIDDriveToPoint {
    private final RobotHardware robot;
    private final double tickPerMM = 0;
    private final double maxTicks = 0;

    //PID controller
    private final PIDController pidx = new PIDController(0, 0, 0);
    private final PIDController pidy = new PIDController(0, 0, 0);
    private final PIDController pidh = new PIDController(0, 0, 0);

    public PIDDriveToPoint(RobotHardware robot) {
        this.robot = robot;
    }
}
