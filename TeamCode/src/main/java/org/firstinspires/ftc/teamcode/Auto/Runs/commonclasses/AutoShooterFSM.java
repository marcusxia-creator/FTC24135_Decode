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
    private final AutoSpindexerContext spindexerContext;
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
    public AutoShooterFSM(RobotHardware robot, AutoSpindexerContext context){
        this.robot = robot;
        this.spindexerContext = context;
        this.pidController = new PIDController(
                PIDTuning.kP,
                PIDTuning.kI,
                PIDTuning.kD
        );
        this.kS = FeedforwardTuning.kS;
        this.kV = FeedforwardTuning.kV;
    }

    ///Shooter Rapid Run Mode
    public static class ShooterRapidRunMode implements Action {
        public enum SHOOTERSTATE {
            SHOOTER_INIT,
            SHOOTER_RUN,
            SHOOTER_SWITCH,
            SHOOTER_LAUNCH,
            SHOOTER_RESET,
            SHOOTER_END;
        }

        /// Variables
        private final RobotHardware robot;
        private AutoSpindexerContext spindexerContext;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime stateTimer2 = new ElapsedTime();
        private final ElapsedTime shooterTimer = new ElapsedTime();

        private  int rapidStartingSlot = 0;
        private int sortingStartingSlot;
        private int targetSlot;
        private double shootSpeed;

        private double ShooterWaitTime;
        private double targetVelocity;

        private final SHOOTERSTATE startingState;
        private final SHOOTERSTATE endState;
        private boolean hasEnteredEndState = false;
        public SHOOTERSTATE currentState;

        public ShooterRapidRunMode(RobotHardware robot, SHOOTERSTATE startingState, SHOOTERSTATE endState, double ShotPower, double shootSpeed, double ShooterWaitTime, AutoSpindexerContext spindexerContext) {
            this.spindexerContext = spindexerContext;
            this.robot = robot;

            this.endState = endState;
            this.startingState = startingState;
            this.currentState = startingState;

            this.targetVelocity = ShotPower * shooterMaxRPM;
            this.ShooterWaitTime = ShooterWaitTime;
            this.sortingStartingSlot = spindexerContext.shootingInitSlot;
            this.shootSpeed = shootSpeed;
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

        public void FSMShooterRapidFire() {
            switch (currentState) {
                case SHOOTER_INIT:
                    spindexerContext.shooterStarted = true;
                    spindexerContext.intakeShouldStop = true;
                    robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
                    robot.kickerServo.setPosition(kickerRetract);
                    SpindexerRunTo(rapidStartingSlot);
                    shooterTimer.reset();
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RUN;
                    break;
                case SHOOTER_RUN:
                    if (stateTimer.seconds() > ShooterWaitTime+0.2) {
                        robot.kickerServo.setPosition(kickerExtend);
                        if (stateTimer.seconds() > ShooterWaitTime + 0.5) {
                            stateTimer2.reset();
                            currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                        }
                    }
                    break;
                case SHOOTER_SWITCH:
                    if (targetSlot <= (rapidStartingSlot + 3)) {
                        targetSlot++;
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                    else {
                        if (stateTimer2.seconds() > 0.5) {
                            stateTimer2.reset();
                            currentState = SHOOTERSTATE.SHOOTER_RESET;
                        }
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
                    robot.kickerServo.setPosition(kickerRetract);
                    if (stateTimer2.seconds() > 0.3) {
                        SpindexerRunTo(1);
                        if(stateTimer2.seconds()>0.6){
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
            SHOOTERSTATE stateBeforeRun = currentState;


            telemetryPacket.put("FSM Shooter State", currentState);

            RunShooter(targetVelocity);
            FSMShooterRapidFire();

            if (stateBeforeRun == endState) {
                hasEnteredEndState = true;
            }

            if (hasEnteredEndState && currentState != endState) {
                return false;
            }

            if (currentState == SHOOTERSTATE.SHOOTER_END) {
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public Action ShootFarZone(double ShotPower, double ShooterWaitTime, ShooterRapidRunMode.SHOOTERSTATE startingState, ShooterRapidRunMode.SHOOTERSTATE endState){
        return new ShooterRapidRunMode(robot, startingState, endState, ShotPower,0.23,ShooterWaitTime, spindexerContext);
    }

    public Action ShootCloseZone(double ShotPower, double ShooterWaitTime, ShooterRapidRunMode.SHOOTERSTATE startingState, ShooterRapidRunMode.SHOOTERSTATE endState){
        return new ShooterRapidRunMode(robot, startingState, endState, ShotPower,0.1,ShooterWaitTime, spindexerContext);
    }

    ///Shooter Sorting Run Mode
    public static class ShooterSortingRunMode implements Action {
        public enum SORTINGSHOOTERSTATE {
            SORTINGSHOOTER_INIT,
            SORTINGSHOOTER_RUN,
            SORTINGSHOOTER_SWITCH,
            SORTINGSHOOTER_LAUNCH,
            SORTINGSHOOTER_EMPTY,
            SORTINGSHOOTER_RESET,
            SORTINGSHOOTER_END;
        }

        /// Variables
        private final RobotHardware robot;
        private AutoSpindexerContext spindexerContext;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime stateTimer2 = new ElapsedTime();
        private final ElapsedTime shooterTimer = new ElapsedTime();

        private int sortingStartingSlot;
        private int targetSlot;
        private double shootSpeed;

        private double ShooterWaitTime;
        private double targetVelocity;

        private final SORTINGSHOOTERSTATE startingState;
        private final SORTINGSHOOTERSTATE endState;
        private boolean hasEnteredEndState = false;
        public SORTINGSHOOTERSTATE currentState;

        public ShooterSortingRunMode(RobotHardware robot, SORTINGSHOOTERSTATE startingState, SORTINGSHOOTERSTATE endState, double ShotPower, double shootSpeed, double ShooterWaitTime, AutoSpindexerContext spindexerContext) {
            this.spindexerContext = spindexerContext;
            this.robot = robot;

            this.endState = endState;
            this.startingState = startingState;
            this.currentState = startingState;

            this.targetVelocity = ShotPower * shooterMaxRPM;
            this.ShooterWaitTime = ShooterWaitTime;
            this.shootSpeed = shootSpeed;
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
                robot.spindexerServo.setPosition(spindexerSlot6);
            }
            if (slot == 6) {
                robot.spindexerServo.setPosition(spindexerFullPos);
            }
        }

        public void FSMShooterSortingFire() {
            switch (currentState) {
                case SORTINGSHOOTER_INIT:
                    spindexerContext.shooterStarted = true;
                    spindexerContext.intakeShouldStop = true;
                    robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
                    robot.kickerServo.setPosition(kickerRetract);
                    spindexerContext.updateShootingInitSlot();
                    sortingStartingSlot = spindexerContext.getShootingInitSlot();
                    targetSlot = sortingStartingSlot;
                    SpindexerRunTo(targetSlot);
                    shooterTimer.reset();
                    stateTimer.reset();
                    currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_RUN;
                    break;
                case SORTINGSHOOTER_RUN:
                    if (stateTimer.seconds() > ShooterWaitTime+0.2) {
                        robot.kickerServo.setPosition(kickerExtend);
                        if (stateTimer.seconds() > ShooterWaitTime + 0.5) {
                            stateTimer2.reset();
                            currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_SWITCH;
                        }
                    }
                    break;
                case SORTINGSHOOTER_SWITCH:
                    if (targetSlot <= 2) {
                        targetSlot++;
                        stateTimer2.reset();
                        currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_LAUNCH;
                    }
                    else if (targetSlot>2 && targetSlot <6) {
                        targetSlot = 6;
                        stateTimer2.reset();
                        currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_EMPTY;
                    }
                    else if (targetSlot == 6){
                        if (stateTimer2.seconds() > 0.3) {
                            stateTimer2.reset();
                            currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_RESET;
                        }
                    }
                    else {
                        if (stateTimer2.seconds() > 0.3) {
                            stateTimer2.reset();
                            currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_RESET;
                        }
                    }
                    break;
                case SORTINGSHOOTER_LAUNCH:
                    SpindexerRunTo(targetSlot);
                    if (stateTimer2.seconds() > shootSpeed) {
                        stateTimer.reset();
                        currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_SWITCH;
                    }
                    break;
                case SORTINGSHOOTER_EMPTY:
                    SpindexerRunTo(targetSlot);
                    if (stateTimer2.seconds() > 0.5) {
                        stateTimer.reset();
                        currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_SWITCH;
                    }
                    break;
                case SORTINGSHOOTER_RESET:
                    SpindexerRunTo(5);
                    if (stateTimer2.seconds() > 0.3) {
                        robot.kickerServo.setPosition(kickerRetract);
                        if(stateTimer2.seconds()>0.8){
                            SpindexerRunTo(1);
                            if(stateTimer2.seconds()>1.4){
                                currentState = SORTINGSHOOTERSTATE.SORTINGSHOOTER_END;
                            }
                        }
                    }
                    break;
                case SORTINGSHOOTER_END:
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
            SORTINGSHOOTERSTATE stateBeforeRun = currentState;
            telemetryPacket.put("Imported Green Position", spindexerContext.currentGreenSlot);
            telemetryPacket.put("Imported Target Green", spindexerContext.targetGreenSlot);
            telemetryPacket.put("Calculated Starting Slot", spindexerContext.shootingInitSlot);
            telemetryPacket.put("Actual Starting Slot", sortingStartingSlot);
            telemetryPacket.put("FSM Shooter State", currentState);

            RunShooter(targetVelocity);
            FSMShooterSortingFire();

            if (stateBeforeRun == endState) {
                hasEnteredEndState = true;
            }

            if (hasEnteredEndState && currentState != endState) {
                return false;
            }

            if (currentState == SORTINGSHOOTERSTATE.SORTINGSHOOTER_END) {
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public Action ShootSorting(double ShotPower, double ShooterWaitTime, ShooterSortingRunMode.SORTINGSHOOTERSTATE startingState, ShooterSortingRunMode.SORTINGSHOOTERSTATE endState){
        return new ShooterSortingRunMode(robot, startingState, endState, ShotPower,0.35,ShooterWaitTime, spindexerContext);
    }

    ///Shooter Speed
    public class ShooterOn implements Action {
        private double targetVelocity;
        private AutoSpindexerContext context;

        public ShooterOn (double shotPower){
            spindexerContext.shooterStarted = true;
            spindexerContext.intakeShouldStop = true;
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