package org.firstinspires.ftc.teamcode.Auto.Runs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.Auto.Runs.BlueSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Autonomous(name = "BlueSideFarAuto", group = "Autonomous")
public class BlueSideFarAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 8, Math.toRadians(0));
    public RobotHardware robot;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.rightGateServo.setPosition(gateDown);
        robot.leftGateServo.setPosition(gateDown);
        robot.pushRampServo.setPosition(rampDownPos);

        Action DriveToShoot1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(PreloadShootingPosition_X,PreloadShootingPosition_Y), Math.toRadians(PreloadShootingPosition_Heading))
                .build();

        Action IntakeSet1Drive1 = drive.actionBuilder(initialPose)
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(-90))
            .build();

        Action IntakeSet1Drive2_1 = drive.actionBuilder(new Pose2d(IntakeSet1Position1_X, IntakeSet1Position1_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(-90))
            .build();

        Action IntakeSet1Drive2_2 = drive.actionBuilder(new Pose2d(IntakeSet1Position2_X,IntakeSet1Position2_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(-90))
            .build();

        Action IntakeSet1Drive2_3 = drive.actionBuilder(new Pose2d(IntakeSet1Position3_X,IntakeSet1Position3_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(-90))
            .build();

        Action DriveToShoot2 = drive.actionBuilder(new Pose2d(IntakeSet1Position4_X,IntakeSet1Position4_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(TransitionLocation_X,TransitionLocation_Y),Math.toRadians(-22.5))
            .strafeToLinearHeading(new Vector2d(ShootingPosition_X,ShootingPosition_Y),Math.toRadians(ShootingPosition_Heading))
            .build();

        Action IntakeSet2Drive1 = drive.actionBuilder(new Pose2d(ShootingPosition_X,ShootingPosition_Y,Math.toRadians(ShootingPosition_Heading)))
            .strafeToLinearHeading(new Vector2d(IntakeSet2Position1_X, IntakeSet2Position1_Y), Math.toRadians(-90))
            .build();

        Action IntakeSet2Drive2_1 = drive.actionBuilder(new Pose2d(IntakeSet1Position2_X, IntakeSet2Position1_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet2Position2_X,IntakeSet2Position2_Y),Math.toRadians(-90))
            .build();

        Action IntakeSet2Drive2_2 = drive.actionBuilder(new Pose2d(IntakeSet2Position2_X,IntakeSet2Position2_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet2Position3_X,IntakeSet2Position3_Y),Math.toRadians(-90))
            .build();

        Action IntakeSet2Drive2_3 = drive.actionBuilder(new Pose2d(IntakeSet1Position3_X,IntakeSet1Position3_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet2Position4_X,IntakeSet2Position4_Y),Math.toRadians(-90))
            .build();

        Action DriveToShoot3 = drive.actionBuilder(new Pose2d(IntakeSet2Position4_X,IntakeSet2Position4_Y,Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(ShootingPosition_X,ShootingPosition_Y),Math.toRadians(ShootingPosition_Heading))
            .build();



        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            DriveToShoot3,
                            ShooterSpeedUp(FarShotPower)
                    ),
                    ShooterRun(FarShotPower,3),
                    IntakeSet1Drive1,
                    new ParallelAction(
                        new SequentialAction(
                            IntakeSet1Drive2_1,
                            IntakeSet1Drive2_2,
                            IntakeSet1Drive2_3
                        ),
                        IntakeRun()
                    ),
                    new ParallelAction(
                            DriveToShoot3,
                        ShooterSpeedUp(CloseShotPower)
                    ),
                    ShooterRun(CloseShotPower,0.5),
                    IntakeSet2Drive1,
                    new ParallelAction(
                            new SequentialAction(
                                    IntakeSet2Drive2_1,
                                    IntakeSet2Drive2_2,
                                    IntakeSet2Drive2_3
                            ),
                            IntakeRun()
                    ),
                    new ParallelAction(
                            DriveToShoot3,
                            ShooterSpeedUp(CloseShotPower)
                    ),
                    ShooterRun(CloseShotPower, 0.5)
                )
            );
        }
    }

    public static class IntakeRunMode implements Action {

        public enum INTAKESTATE {
            INTAKE_INIT,
            INTAKE_READY,
            INTAKE_RUN,
            INTAKE_DETECT,
            INTAKE_PAUSE,
            INTAKE_INDEX,
            INTAKE_SPIN,
            INTAKE_UNJAM,
            INTAKE_END

        }

        /// Variables
        private final RobotHardware robot;
        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime colorSensorTimer = new ElapsedTime();
        private INTAKESTATE currentState = INTAKESTATE.INTAKE_READY;
        private int targetSlot = 0;

        /// Constructor
        public IntakeRunMode(RobotHardware robot) {
            this.robot = robot;
            this.currentState = INTAKESTATE.INTAKE_READY;
        }

        public void SpindexerRunTo(int slot){
            if(slot==0){
                robot.spindexerServo.setPosition(spindexerSlot0);
            }
            if(slot==1){
                robot.spindexerServo.setPosition(spindexerSlot1);
            }
            if(slot==2){
                robot.spindexerServo.setPosition(spindexerSlot2);
            }
        }

        public void FSMIntakeRun() {
            switch (currentState) {
                case INTAKE_INIT:
                    SpindexerRunTo(0);
                    robot.leftGateServo.setPosition(gateUp);
                    robot.rightGateServo.setPosition(gateUp);
                    currentState = INTAKESTATE.INTAKE_READY;
                    break;
                case INTAKE_READY:
                    robot.leftGateServo.setPosition(gateUp);
                    robot.rightGateServo.setPosition(gateUp);
                    currentState = INTAKESTATE.INTAKE_RUN;
                    break;
                case INTAKE_RUN:
                    robot.intakeMotor.setPower(0.8);
                    currentState = INTAKESTATE.INTAKE_DETECT;
                    break;
                case INTAKE_DETECT:
                    //if (targetSlot >= 3){currentState = INTAKESTATE.INTAKE_END;}
                    if (/**robot.intakeMotor.getVelocity()<=intakeRPM_THRESHOLD||*/robot.distanceSensor.getDistance(DistanceUnit.MM) < 50) {
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_PAUSE;
                    } else {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_PAUSE:
                    robot.intakeMotor.setPower(intakeStop);
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                    if (stateTimer.seconds()>0.2) {
                        targetSlot++;
                        currentState = INTAKESTATE.INTAKE_INDEX;
                    }
                    break;
                case INTAKE_INDEX:
                    if (targetSlot <= 2){
                        //need to set spindexer to spin here.
                        SpindexerRunTo(targetSlot);
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_SPIN;
                    }else if(targetSlot >= 3){
                        currentState = INTAKESTATE.INTAKE_END;
                    }
                    break;
                case INTAKE_SPIN:
                    if (stateTimer.seconds()>0.2){
                        currentState = INTAKESTATE.INTAKE_READY;
                    }
                    break;
                case INTAKE_UNJAM:
                    robot.intakeMotor.setPower(ejectSpeed);
                    currentState = INTAKESTATE.INTAKE_RUN;
                    break;
                case INTAKE_END:
                    robot.intakeMotor.setPower(intakeStop);
                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put("FSM Intake State", currentState);
            FSMIntakeRun();
                return currentState != INTAKESTATE.INTAKE_END;
        }
    }

    public Action IntakeRun(){
        return new IntakeRunMode(robot);
    }

    public static class ShooterRunMode implements Action {
        public enum SHOOTERSTATE {
            SHOOTER_INIT,
            SHOOTER_RUN,
            SHOOTER_SWITCH,
            SHOOTER_LAUNCH,
            SHOOTER_RESET_1,
            SHOOTER_RESET_2,
            SHOOTER_END
        }

        ///Variables
        private final RobotHardware robot;
        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime stateTimer2 = new ElapsedTime();
        private final ElapsedTime colorSensorTimer = new ElapsedTime();
        private int targetSlot = 2;
        private double ShooterWaitTime;
        private double ShotPower;
        private SHOOTERSTATE currentState;

        public ShooterRunMode (RobotHardware robot, double ShotPower, double ShooterWaitTime){
            this.robot = robot;
            this.ShotPower = ShotPower;
            this.ShooterWaitTime = ShooterWaitTime;
            this.currentState = SHOOTERSTATE.SHOOTER_INIT;
        }

        public void SpindexerRunTo(int slot){
            if(slot==0){
                robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
            }
            if(slot==1){
                robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
            }
            if(slot==2){
                robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
            }
        }

        public void FSMShooterRun() {
            switch (currentState) {
                case SHOOTER_INIT:
                    robot.pushRampServo.setPosition(rampDownPos);
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RUN;
                    break;
                case SHOOTER_RUN:
                    SpindexerRunTo(targetSlot);
                    robot.shooterMotor.setPower(ShotPower);
                    if (stateTimer.seconds()>ShooterWaitTime){
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                    break;
                case SHOOTER_LAUNCH:
                    robot.leftGateServo.setPosition(gateUp);
                    robot.rightGateServo.setPosition(gateUp);
                    if (stateTimer2.seconds()>0.4) {
                        robot.pushRampServo.setPosition(rampUpPos);
                        stateTimer.reset();
                        currentState = SHOOTERSTATE.SHOOTER_RESET_1;
                    }
                    break;
                case SHOOTER_RESET_1:
                    if (stateTimer.seconds()>0.5) {
                        robot.pushRampServo.setPosition(rampDownPos);
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_RESET_2;
                    }
                    break;
                case SHOOTER_RESET_2:
                    if (stateTimer2.seconds()>0.3) {
                        robot.leftGateServo.setPosition(gateDown);
                        robot.rightGateServo.setPosition(gateDown);
                        targetSlot--;
                        stateTimer.reset();
                        currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                    }
                    break;
                case SHOOTER_SWITCH:
                    if (targetSlot >=0){
                        //need to set spindexer to spin here.
                        SpindexerRunTo(targetSlot);
                        if (stateTimer.seconds()>0.7) {
                            stateTimer2.reset();
                            currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                        }
                    }else if(targetSlot <0){
                        currentState = SHOOTERSTATE.SHOOTER_END;
                    }
                    break;
                case SHOOTER_END:
                    robot.shooterMotor.setPower(0);
                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put("FSM Intake State", currentState);
            FSMShooterRun();
            return currentState != SHOOTERSTATE.SHOOTER_END;
        }
    }

    public Action ShooterRun(double ShotPower, double ShooterWaitTime){
        return new ShooterRunMode(robot, ShotPower, ShooterWaitTime);
    }

public static class ShooterSpeedUp implements Action {
    private double TargetSpeed;
    private final RobotHardware robot;

    public ShooterSpeedUp (RobotHardware robot, double TargetSpeed){
        this.robot = robot;
        this.TargetSpeed = TargetSpeed;
    }

    public void StartShooter (){
        robot.intakeMotor.setPower(TargetSpeed);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        StartShooter();
        return false;
    }
}

    public Action ShooterSpeedUp (double TargetShotPower){
        return new ShooterSpeedUp(robot,TargetShotPower);
    }

}
