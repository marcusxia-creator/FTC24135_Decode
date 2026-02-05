package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static java.lang.Math.floorMod;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class TurretRunMode implements Action{
    public final RobotHardware robot;

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1 / tickToAngle;

    private final int targetAngle;

    public TurretRunMode (RobotHardware robot, int targetAngle) {
        this.robot = robot;
        this.targetAngle = targetAngle;
    }

    public void TurretRunTo (){
        int ticks = (int)(Range.clip(targetAngle, -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        TurretRunTo();
        return false;
    }
}
