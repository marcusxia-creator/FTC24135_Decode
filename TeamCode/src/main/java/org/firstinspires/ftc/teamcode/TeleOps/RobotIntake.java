package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotIntake {

    //Declare intake states
    private IntakeState intakeState;

    //Declare gamepad
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    //Declare the gamepads for the rising edge detector
    //Gamepad1
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad1 = new Gamepad();
    //Gamepad2
    private final Gamepad previousGamepad2 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();

    private final RobotHardware robot;

    //Set up timer for debouncing
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = RobotActionConfig.DEBOUNCE_THRESHOLD;

    private final ElapsedTime intakeTimer = new ElapsedTime();

    //Constructor
    public RobotIntake (Gamepad gamepad1, Gamepad gamepad2, RobotHardware robot) {

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;

        this.intakeState = IntakeState.INTAKE_EXTEND;
    }

    public void intakeSlideControl () {
        switch (intakeState) {
            case INTAKE_EXTEND:
                if ((gamepad1.dpad_right || gamepad2.dpad_right) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Extend);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Extend);
                    robot.intakeSlideServo.setPosition(RobotActionConfig.intake_Slide_Extend);
                    intakeState = IntakeState.INTAKE_GRAB;
                }
                else {
                    intakeInit();
                }
                break;
            case INTAKE_GRAB:
                if ((gamepad1.dpad_left || gamepad2.dpad_left) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Extend);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Extend);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Default);
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                else {
                    //Assigning value to each gamepad
                    previousGamepad1.copy(currentGamepad1);
                    currentGamepad1.copy(gamepad1);

                    previousGamepad2.copy(currentGamepad2);
                    currentGamepad2.copy(gamepad2);

                    //Calling the methods
                    intakeArmControl();
                    intakeRotationServoSteer();
                }
                break;
            case INTAKE_RETRACT:
                robot.intakeSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                if (intakeTimer.seconds() > 0.5) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Retract);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Retract);
                    intakeState = IntakeState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER:
                if(intakeTimer.seconds() > 1) {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                }
                if(intakeTimer.seconds() > 1.3) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                }
                if(intakeTimer.seconds() > 1.5) {
                    intakeTimer.reset();
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    RobotActionConfig.depositState = RobotActionConfig.DepositState.DEPOSIT_EXTEND;
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                break;
            default:
                intakeState = IntakeState.INTAKE_EXTEND;
                break;
        }
    }

    private enum IntakeState {
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        SAMPLE_TRANSFER
    }

    public void intakeInit () {
        robot.intakeSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Default);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
    }

    private void intakeArmControl () {
        //Rising edge detector
        if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() + RobotActionConfig.intake_Arm_Change_Amount);
        }
        if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() - RobotActionConfig.intake_Arm_Change_Amount);
        }
    }

    private void intakeRotationServoSteer() {
        //Rising edge detector
        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() + RobotActionConfig.intake_Rotation_Steer_Amount);
        }
        if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) || (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() - RobotActionConfig.intake_Rotation_Steer_Amount);
        }
    }
}
