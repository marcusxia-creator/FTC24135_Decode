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
public class AutoShooterFSM {
    private final RobotHardware robot;
    private static PIDController pidController;

    //Feedforward variables
    private static double kS; // static friction
    private static double kV; // velocity coefficient
    public static final double tickToRPM = 60.0 / 28.0;

    public static class PIDTuning {
        public static double kP = 2;
        public static double kI = 0;
        public static double kD = 0.02; // position or RPM target
    }

    public static class FeedforwardTuning {
        public static double kS = 0.03;  // static friction (small bump)
        public static double kV = 1.0;  // scale from targetNorm to power (roughly 1.0 if perfect)
    }

    ///Constructor
    public AutoShooterFSM(RobotHardware robot){
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

        private int startingSlot;
        private int targetSlot;
        private final int currentGreenSlot;
        private final int targetGreenSlot;

        private final double shootSpeed;

        private double ShooterWaitTime;
        private double targetVelocity;

        public SHOOTERSTATE currentState;

        private void updateShootingInitSlot() {
            if (currentGreenSlot == -1) {
                startingSlot = 0;
            } else {
                startingSlot = Math.floorMod(currentGreenSlot - targetGreenSlot, 3);
            }
        }

        public ShooterRunMode(RobotHardware robot, double ShotPower, double shootSpeed, double ShooterWaitTime, int currentGreenSlot, int targetGreenSlot) {
            this.robot = robot;
            ///Timing
            this.targetVelocity = ShotPower*shooterMaxRPM;
            this.ShooterWaitTime = ShooterWaitTime;
            this.currentState = SHOOTERSTATE.SHOOTER_INIT;
            this.shootSpeed = shootSpeed;
            ///Calculate Sorting
            this.currentGreenSlot = currentGreenSlot;
            this.targetGreenSlot = targetGreenSlot;
            updateShootingInitSlot();
            this.targetSlot = startingSlot;
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
            if (slot == 3){
                robot.spindexerServo.setPosition(spindexerSlot4);
            }
            if (slot == 4){
                robot.spindexerServo.setPosition(spindexerSlot5);
            }
            if (slot == 5) {
                robot.spindexerServo.setPosition(spindexerFullPos);
            }
        }

        public void FSMShooterRun() {
            switch (currentState) {
                case SHOOTER_INIT:
                    robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
                    robot.kickerServo.setPosition(kickerRetract);
                    SpindexerRunTo(startingSlot);
                    shooterTimer.reset();
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RUN;
                    break;
                case SHOOTER_RUN:
                    if (stateTimer.seconds() > ShooterWaitTime+0.3) {
                        robot.kickerServo.setPosition(kickerExtend);
                        if (stateTimer.seconds() > ShooterWaitTime + 0.6) {
                            stateTimer2.reset();
                            currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                        }
                    }
                    break;
                case SHOOTER_SWITCH:
                    if (targetSlot <= (startingSlot + 3)) {
                        targetSlot++;
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                    else {
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_RESET;
                    }
                    break;
                case SHOOTER_LAUNCH:
                    SpindexerRunTo(targetSlot);
                    if (stateTimer2.seconds() > shootSpeed) {
                        stateTimer.reset();
                        currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                    }
                    break;
                case SHOOTER_RESET:
                    SpindexerRunTo(0);
                    if (stateTimer2.seconds() > 0.4) {
                        robot.kickerServo.setPosition(kickerRetract);
                        if(stateTimer2.seconds()>0.7){
                            currentState = SHOOTERSTATE.SHOOTER_END;
                        }
                    }
                    break;
                case SHOOTER_END:
                    robot.topShooterMotor.setPower(0);
                    robot.bottomShooterMotor.setPower(0);
                    break;
            }
        }

        public void RunShooter(double targetVelocity){
            double currentRPM = robot.topShooterMotor.getVelocity() * tickToRPM;
            double voltage  = robot.getBatteryVoltageRobust();
            double maxRPMDynamic = shooterMaxRPM * voltage /REF_VOLTAGE;
            //Normalised current and max velocity to 0..1 for stable tuning
            double normCurrentRPM = clamp01(currentRPM/maxRPMDynamic);
            double normTargetRPM = clamp01(targetVelocity /maxRPMDynamic);//Target velocity
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
                ///Dashboard Telemetry
                telemetryPacket.put("Actual Starting Slot", startingSlot);
                telemetryPacket.put("FSM Shooter State", currentState);
                ///Run Shooter & FSM
                RunShooter(targetVelocity);
                FSMShooterRun();
                return true;
            }
        }
    }

    public Action ShootFarZone(double ShotPower, double ShooterWaitTime, int currentGreen, int targetGreen){
        return new ShooterRunMode(robot, ShotPower,0.5,ShooterWaitTime,currentGreen,targetGreen);
    }

    public Action ShootCloseZone (double ShotPower, double ShooterWaitTime, int currentGreen, int targetGreen){
        return new ShooterRunMode(robot, ShotPower,0.1, ShooterWaitTime, currentGreen, targetGreen);
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

    private static double clamp01(double x) { return x<0?0:(x>1?1:x); }

}