package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Config
public class Shooter {
    private final RobotHardware robot;
    private static PIDController pidController;

    //Feewforward variables
    private static double kS; // static friction
    private static double kV; // velocity coefficient
    public static final double tickToRPM = 60.0 / 28.0;

    public static class PIDTuning {
        public static double kP = 10;
        public static double kI = 0;
        public static double kD = 0.7; // position or RPM target
    }

    public static class FeedforwardTuning {
        public static double kS = 0.03;  // static friction (small bump)
        public static double kV = 1.285;  // scale from targetNorm to power (roughly 1.0 if perfect)
    }

    ///Constructor
    public Shooter(RobotHardware robot){
        this.robot = robot;
        this.pidController = new PIDController(
                PIDTuning.kP,
                PIDTuning.kI,
                PIDTuning.kD
        );
        this.kS = FeedforwardTuning.kS;
        this.kV = FeedforwardTuning.kV;
    }

    ///Shooter Run Mode
    public static class ShooterRunMode implements Action {
        public enum SHOOTERSTATE {
            SHOOTER_INIT,
            SHOOTER_RUN,
            SHOOTER_SWITCH,
            SHOOTER_LAUNCH,
            SHOOTER_RESET,
            SHOOTER_END
        }

        /// Variables
        private final RobotHardware robot;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime stateTimer2 = new ElapsedTime();
        private final ElapsedTime shooterTimer = new ElapsedTime();

        private int startingSlot = 3;
        private int targetSlot = startingSlot;
        private double ShooterWaitTime;
        private double targetVelocity;
        public SHOOTERSTATE currentState;

        public ShooterRunMode(RobotHardware robot, double ShotPower, double ShooterWaitTime) {
            this.robot = robot;
            this.targetVelocity = ShotPower*shooterMaxRPM;
            this.ShooterWaitTime = ShooterWaitTime;
            this.currentState = SHOOTERSTATE.SHOOTER_INIT;
        }

        public void SpindexerRunTo(int slot) {
            if (slot == 0) {
                robot.spindexerServo.setPosition(spindexerZeroPos);
            }
            if (slot == 1) {
                robot.spindexerServo.setPosition(spindexerSlot1);
            }
            if (slot == 2) {
                robot.spindexerServo.setPosition(spindexerSlot2);
            }
            if (slot == 3) {
                robot.spindexerServo.setPosition(spindexerSlot3);
            }
            if (slot == 4){
                robot.spindexerServo.setPosition(spindexerSlot4);
            }
            if (slot == 5){
                robot.spindexerServo.setPosition(spindexerSlot5);
            }
        }

        public void FSMShooterRun() {
            switch (currentState) {
                case SHOOTER_INIT:
                    robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
                    SpindexerRunTo(targetSlot);
                    robot.kickerServo.setPosition(kickerRetract);
                    shooterTimer.reset();
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RUN;
                    break;
                case SHOOTER_RUN:
                    if (stateTimer.seconds() > ShooterWaitTime) {
                        robot.kickerServo.setPosition(kickerExtend);
                        currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                    }
                    break;
                case SHOOTER_SWITCH:
                    if (targetSlot == 0 || targetSlot == (startingSlot - 3)) {
                        stateTimer.reset();
                        currentState = SHOOTERSTATE.SHOOTER_RESET;
                    }
                    else {
                        targetSlot--;
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                    break;
                case SHOOTER_LAUNCH:
                    if (stateTimer2.seconds() > 0.2) {
                        SpindexerRunTo(targetSlot);
                        if (stateTimer2.seconds() > 0.3) {
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

        public void RunShooter(double targetRPM){
            double currentRPM = robot.topShooterMotor.getVelocity() * tickToRPM;

            //Normalised current and max velocity to 0..1 for stable tuning
            double normCurrentRPM = currentRPM/shooterMaxRPM;
            double normTargetRPM = targetRPM /shooterMaxRPM; //Target velocity

            //Feedforward calculations
            double ff = (kS * Math.signum(normTargetRPM)) + (kV * normTargetRPM);
            //PID calculations
            double pidPower = pidController.calculate(normCurrentRPM, normTargetRPM);

            //Shooter total power
            double power = ff + pidPower;

            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (currentState == SHOOTERSTATE.SHOOTER_END) {
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                return false;
            }
            else {
                RunShooter(targetVelocity);
                FSMShooterRun();

                return true;
            }
        }
    }

    public Action ShooterRun(double ShotPower, double ShooterWaitTime){
        return new ShooterRunMode(robot, ShotPower, ShooterWaitTime);
    }

    ///Shooter Speed
    public class ShooterOn implements Action {
        private double targetVelocity;

        public ShooterOn (double shotPower){
            this.targetVelocity = shotPower*shooterMaxRPM;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentVel = robot.topShooterMotor.getVelocity()*tickToRPM;
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