package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.controller.PIDController;

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

}
