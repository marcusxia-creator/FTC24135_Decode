package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "TrajectoryTest", group = "Autonomous")
public class TrajectoryTest extends LinearOpMode {

    public static Pose2d initialPose = new Pose2d(64, 8, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        RobotHardware robot = new RobotHardware(hardwareMap);
        RobotActionConfig robotActionConfig = new RobotActionConfig();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            .strafeToLinearHeading(new Vector2d(36, 32), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(36, 48), Math.toRadians(90))
                            .stopAndAdd(new IntakeRunMode(robot,robotActionConfig, FSMIntakeAuto.INTAKESTATE.INTAKE_READY))
                            .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(22.5))
                            .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(-45))
                            .strafeToLinearHeading(new Vector2d(12, 32), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(12, 48), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(-45))
                            .strafeToLinearHeading(new Vector2d(10, 12), Math.toRadians(-90))
                            .build());
        }
    }
    public static class IntakeRunMode implements Action {

        public enum INTAKESTATE {
            INTAKE_READY,
            INTAKE_RUN,
            INTAKE_DETECT,
            INTAKE_INDEX,
            INTAKE_SPIN,
            INTAKE_END,
            INTAKE_UNJAM
        }

        /// Variables
        private FSMIntakeAuto.INTAKESTATE currentState;
        private final RobotHardware robot;
        private final RobotActionConfig robotActionConfig;
        private final ElapsedTime stateTimer = new ElapsedTime();

        /// Constructor
        public IntakeRunMode(RobotHardware robot, RobotActionConfig robotActionConfig, FSMIntakeAuto.INTAKESTATE currentState) {
            this.robot = robot;
            this.robotActionConfig = robotActionConfig;
            this.currentState = currentState;
        }

        public void FSMRun() {
            switch (currentState) {
                case INTAKE_READY:
                    robot.init();
                    break;
                case INTAKE_RUN:
                    robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);

                    break;
                case INTAKE_DETECT:
                    break;
                case INTAKE_INDEX:
                    break;
                case INTAKE_SPIN:
                    break;
                case INTAKE_UNJAM:
                    break;
                case INTAKE_END:
                    break;

            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FSMRun();
            return false;
        }

    }
}
