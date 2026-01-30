package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class Shooter {
    private final RobotHardware robot;
    private static PIDController pidController;
    public ShooterRunMode.SHOOTERSTATE globalState = ShooterRunMode.SHOOTERSTATE.SHOOTER_INIT;

    public static class PIDTuning {
        public static double kP = 0.0075;
        public static double kI = 0.000001;
        public static double kD = 0.00006; // position or RPM target
    }

    ///Constructor
    public Shooter(RobotHardware robot){
        this.robot = robot;
        this.pidController = new PIDController(
                PIDTuning.kP,
                PIDTuning.kI,
                PIDTuning.kD
        );
    }

    ///Shooter Run Mode
    public static class ShooterRunMode implements Action {
        public enum SHOOTERSTATE {
            SHOOTER_INIT,
            SHOOTER_RUN,
            SHOOTER_LAUNCH,
            SHOOTER_RESET,
            SHOOTER_END
        }

        /// Variables
        private final RobotHardware robot;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime stateTimer2 = new ElapsedTime();
        private final ElapsedTime shooterTimer = new ElapsedTime();

        private int targetSlot = 2;
        private double ShooterWaitTime;
        private double targetVelocity;
        public SHOOTERSTATE currentState;

        public ShooterRunMode(RobotHardware robot, double ShotPower, double ShooterWaitTime) {
            this.robot = robot;
            this.targetVelocity = ShotPower * shooterMaxVel;
            this.ShooterWaitTime = ShooterWaitTime;
            this.currentState = SHOOTERSTATE.SHOOTER_INIT;
        }

        public void SpindexerRunTo(int slot) {
            if (slot == 0) {
                robot.spindexerServo.setPosition(spindexerSlot1);
            }
            if (slot == 1) {
                robot.spindexerServo.setPosition(spindexerSlot2);
            }
            if (slot == 2) {
                robot.spindexerServo.setPosition(spindexerSlot3);
            }
            if (slot == 3) {
                robot.spindexerServo.setPosition(spindexerSlot4);
            }
            if (slot == 4){
                robot.spindexerServo.setPosition(spindexerSlot5);
            }
        }

        public void FSMShooterRun() {
            switch (currentState) {
                case SHOOTER_INIT:
                    SpindexerRunTo(targetSlot);
                    robot.kickerServo.setPosition(kickerRetract);
                    shooterTimer.reset();
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RUN;
                    break;
                case SHOOTER_RUN:
                    if (stateTimer.seconds() > ShooterWaitTime) {
                        robot.kickerServo.setPosition(kickerExtend);
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                    break;
                case SHOOTER_LAUNCH:
                    if (stateTimer2.seconds() > 0.2) {
                        robot.spindexerServo.setPosition(spindexerZeroPos);
                        if (stateTimer2.seconds() > 0.8) {
                            stateTimer.reset();
                            currentState = SHOOTERSTATE.SHOOTER_RESET;
                        }
                    }
                    break;
                case SHOOTER_RESET:
                    robot.spindexerServo.setPosition(spindexerSlot1);
                    if (stateTimer.seconds() > 0.2) {
                        robot.kickerServo.setPosition(kickerRetract);
                        if(stateTimer.seconds()>0.3){
                            currentState = SHOOTERSTATE.SHOOTER_END;
                        }
                    } else if (shooterTimer.seconds()>(8+ShooterWaitTime)) {
                        currentState = SHOOTERSTATE.SHOOTER_END;
                    }
                    break;
                case SHOOTER_END:
                    robot.topShooterMotor.setPower(0);
                    robot.bottomShooterMotor.setPower(0);
                    break;
            }
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (currentState != SHOOTERSTATE.SHOOTER_END){
                double currentVel = robot.topShooterMotor.getVelocity();
                double power = pidController.calculate(currentVel, targetVelocity);
                robot.topShooterMotor.setPower(power);
                robot.bottomShooterMotor.setPower(power);
            }
            FSMShooterRun();
            return currentState != SHOOTERSTATE.SHOOTER_END;
        }
    }

    public Action ShooterRun(double ShotPower, double ShooterWaitTime){
        return new ShooterRunMode(robot, ShotPower, ShooterWaitTime);
    }

    ///Shooter Speed
    public class ShooterOn implements Action {
        private double targetVelocity;

        public ShooterOn (double shotPower){
            this.targetVelocity = shotPower * shooterMaxVel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentVel = robot.topShooterMotor.getVelocity();
            double power = pidController.calculate(currentVel, targetVelocity);
            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
            return false;
        }
    }

    public Action ShooterOn (double shotPower){
        return new ShooterOn(shotPower);
    }

    ///Shooter Off
    public class ShooterOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.topShooterMotor.setPower(0);
            robot.bottomShooterMotor.setPower(0);
            return false;
        }
    }

    public Action ShooterOff (){
        return new ShooterOff();
    }

}