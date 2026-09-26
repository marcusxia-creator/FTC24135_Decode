package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class ShooterSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private double shooterPower = 0.5;
    private double POWER_STEP = 0.1;
    public ShooterSubsystem (RobotHardware robot){
        this.robot = robot;
    }
    public void runFlywheel (){robot.shooterMotor.setPower(shooterPower);}
    public void increasePower (){Math.min (1.0, shooterPower + POWER_STEP);}
    public void decreasePower (){Math.max(0.0, shooterPower + POWER_STEP);}
    public double getshooterPower (){return shooterPower; }
    public void stop (){robot.shooterMotor.setPower(0);}
}
