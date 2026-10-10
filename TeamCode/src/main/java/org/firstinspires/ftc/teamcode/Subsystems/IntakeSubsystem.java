package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.Subsystems.UniversalTools.RobotHardware;

public class IntakeSubsystem extends SubsystemBase{
    private RobotHardware robot;
    private double intakePower = 0.7;
    private final double POWER_STEP = 0.1;
    public IntakeSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void runRollers (){
        robot.intakeMotor.setPower(intakePower);
    }
    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + POWER_STEP);
    }

    public void decreasePower (){
        intakePower = Math.max(0.0, intakePower - POWER_STEP);
    }
    public double getIntakePower() {
        return intakePower;
    }

    public void stop () {
        robot.intakeMotor.setPower(0);
    }

    public class RunIntake implements Action{
        public RunIntake(){}

        public void init(){
            runRollers();
        }

        @Override
        public void loop() {}

        @Override
        public void shutdown(){
            stop();
        }
    }
}
/*
@TeleOp(name="intakeTest")
class test extends OpMode{
    IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(new RobotHardware(hardwareMap));
    }

    @Override
    public void loop() {
        intake.runRollers();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
*/
