package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotHardware;

import CommandBase.Action;
import CommandBase.PrebuiltActions.*;

public class Intake {
    DcMotorEx motor;
    public Action runIntake;

    public Intake(RobotHardware robot){
        motor=robot.intakeMotor;
        runIntake=new ActionSeries(new runMotor(motor));//Written as a series to allow for more actions in series
    }

    public class runMotor implements Action{
        DcMotorEx motor;
        public runMotor(DcMotorEx motor){
            this.motor=motor;
        }

        @Override
        public void init() {
            motor.setPower(1);
        }

        @Override
        public void loop() {}

        @Override
        public void shutdown(){
            motor.setPower(0);
        }

        @Override
        public boolean finished() {
            return false;
        }
    }
}
