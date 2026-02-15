package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;


import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Config
public class AutoTurretDrive {
    private final RobotHardware robot;
    private PIDController pidController;

    public final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    public final double angleToTick = 1 / tickToAngle;

    public static double kP = 0.002, kI = 0, kD = 0.0003, kS = 0.0001, kV = 0.005;

    public AutoTurretDrive(RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(kP, kI, kD);
    }

    public double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }

    public class TurretRunTo implements Action {
        private final int targetAngle;

        public TurretRunTo (int targetAngle) {
            this.targetAngle = targetAngle;
        }

        public void runToPos (int targetAngle) {
            int targetTicks = (int) (Range.clip(targetAngle,-180, 180) * angleToTick);
            int currentTicks = robot.turretMotor.getCurrentPosition();
            int errorTicks = targetTicks - currentTicks;
            double ff = (kS * Math.signum(errorTicks)) + (kV * errorTicks);
            double power = pidController.calculate(currentTicks, targetTicks);
            double output = power + ff;
            robot.turretMotor.setTargetPosition(targetTicks);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runToPos(targetAngle);
            return false;
        }
    }

    public Action TurretRun (int targetAngle) {
        return new TurretRunTo(targetAngle);
    }

    ///Auto Aiming --- Incomplete
    /*
    public class TurretAutoAim implements Action {
        private final Pose2d goalPose;
        private final Pose2d currentPose;

        private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);
        private double conversionFactor = 39.3700787;
        private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);

        public TurretAutoAim (Pose2d goalPose, Pose2d currentPose) {
            this.goalPose = goalPose;
            this.currentPose = currentPose;
        }

        public double getTargetAngle () {
            // NEW -- Fail-safe: if goalPose somehow isn't set, don't spin
            if (goalPose == null) return Math.toDegrees(currentPose.heading.real);
            double turretYaw = THETA + currentPose.heading.real;
            double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            double turretcentX = currentPose.position.x + turretXOffSet;
            double turretcentY = currentPose.position.y + turretYOffSet;
            return Math.toDegrees(Math.atan2((goalPose.position.y-turretcentY), (goalPose.position.x-turretcentX)));
        }

        public void runToAutoAim () {
            int targetTicks = (int) (Range.clip(getTargetAngle(),-180, 180) * angleToTick);
            int currentTicks = robot.turretMotor.getCurrentPosition();
            int errorTicks = targetTicks - currentTicks;
            double ff = (kS * Math.signum(errorTicks)) + (kV * errorTicks);
            double power = pidController.calculate(currentTicks, targetTicks);
            double output = power + ff;
            robot.turretMotor.setTargetPosition(targetTicks);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runToAutoAim();
            return false;
        }
    }

     */
}
