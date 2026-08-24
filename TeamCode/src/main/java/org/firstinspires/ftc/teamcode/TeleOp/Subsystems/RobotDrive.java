package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotHardware;

import java.util.function.Supplier;

import CommandBase.Action;


public class RobotDrive {
    //Container for manual and autodrive, although autodrive might be split out entirely
    RobotHardware robot;
    public ManualDrive manualDrive;

    public RobotDrive(RobotHardware robot, Supplier<Double> driveX, Supplier<Double> driveY, Supplier<Double> driveRot){
        this.robot=robot;
        manualDrive=new ManualDrive(robot,driveX,driveY,driveRot);
    }

    public class ManualDrive implements Action {
        DcMotorEx FL;
        DcMotorEx FR;
        DcMotorEx BL;
        DcMotorEx BR;
        Supplier<Double> driveX;
        Supplier<Double> driveY;
        Supplier<Double> driveRot;

        public ManualDrive(RobotHardware robot, Supplier<Double> driveX, Supplier<Double> driveY, Supplier<Double> driveRot){
            this.FL=robot.frontLeftMotor;
            this.FR=robot.frontRightMotor;
            this.BL=robot.backLeftMotor;
            this.BR=robot.backRightMotor;

            this.driveX=driveX;
            this.driveY=driveY;
            this.driveRot=driveRot;
        }

        @Override
        public void init() {}

        @Override
        public void loop() {
            double x=driveX.get();
            double y=driveY.get();
            double rot=driveRot.get();
            FL.setPower(-y+x-rot);
            FR.setPower(y+x-rot);
            BL.setPower(-y-x-rot);
            BR.setPower(y-x-rot);
        }

        @Override
        public void shutdown() {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

        @Override
        public boolean finished() {
            return false;
        }
    }
}
