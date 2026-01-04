package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Turret {

    private final RobotHardware robot;
    // Target angle to goal
    private final double targetAngle = 135.0;
    //PID constants
    private final double p = 0.01, i = 0.0, d = 0.0;
    private PIDController pidController;
    public Turret(RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(p, i, d);
    }

    public void turretDrive() {
        //robot heading from pinpoint in degrees
        double robotHeading = robot.pinpoint.getHeading(AngleUnit.DEGREES);

        //find the desired turret angle by subtracting robot heading from target angle (goal)
        double desiredTurretAngle = targetAngle - robotHeading;

        //normalize the angle to be between -180 and 180 degrees so the turret goes the shortest path
        desiredTurretAngle = normalizeDegrees(desiredTurretAngle);

        //get the turret IMU's rotation in ZYX
        Orientation turretOrientation = robot.external_imu.getAngularOrientation();
        //get the turret IMU's rotation in Z, or heading (first angle)
        double turretAngle = turretOrientation.firstAngle;

        //error normalized for PID to drive
        double error = normalizeDegrees(desiredTurretAngle - turretAngle);
        //PID calculations for power
        double power = pidController.calculate(error);

        //Run turret motor using PID power
        robot.turretMotor.setPower(power);
    }
}
