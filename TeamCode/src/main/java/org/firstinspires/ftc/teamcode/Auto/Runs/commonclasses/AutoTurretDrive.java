package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Config
public class AutoTurretDrive {
    private final RobotHardware robot;

    public final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    public final double angleToTick = 1 / tickToAngle;

    public AutoTurretDrive(RobotHardware robot) {
        this.robot = robot;
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
            int ticks = (int)(Range.clip(targetAngle, -180, 180) * angleToTick);
            robot.turretMotor.setTargetPosition(ticks);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setPower(1);

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
