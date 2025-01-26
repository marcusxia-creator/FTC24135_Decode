package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RobotIntake {
    //Declare IntakeState
    public enum IntakeState {
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        SAMPLE_TRANSFER
    }

    public void intakeInit () {
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Initial);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Idle);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
    }
    
    //Declare intake states
    public IntakeState intakeState = IntakeState.INTAKE_EXTEND;

    //Declare gamepadEx
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    //Declare gamepad
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    private final RobotHardware robot;

    //Set up timer for debouncing
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final ElapsedTime intakeTimer = new ElapsedTime();

    //Constructor
    public RobotIntake (RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void intakeSlideControl () {
        switch (intakeState) {
            case INTAKE_EXTEND:
                if (((gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)
                        || (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))
                        && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {

                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Idle);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extend);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extend);
                    intakeState = IntakeState.INTAKE_GRAB;
                }
                break;
            case INTAKE_GRAB:
                if ((gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT)) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    intakeTimer.reset();
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                    if (intakeTimer.seconds() > RobotActionConfig.intake_Claw_Grab_Threshold) {
                        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Idle);
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_RETRACT;
                    }
                }
                else {
                    //Declaring the rising edge detector
                    previousGamepad1.copy(currentGamepad1);
                    previousGamepad2.copy(currentGamepad2);

                    currentGamepad1.copy(gamepad1);
                    currentGamepad2.copy(gamepad2);

                    //Calling the methods
                    intakeArmControl();
                    intakeRotationServoSteer();
                }
                break;
            case INTAKE_RETRACT:
                if (intakeTimer.seconds() > RobotActionConfig.intake_Slide_Retract_Threshold) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                }
                if (intakeTimer.seconds() > RobotActionConfig.intake_Wrist_Arm_Retract_Threshold) {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    intakeState = IntakeState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER:
                if (intakeTimer.seconds() > RobotActionConfig.deposit_Claw_Close_Threshold) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                break;
            default:
                intakeState = IntakeState.INTAKE_EXTEND;
                break;
        }

    if (((gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6)
            || (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6))
            && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {

            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
            robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
            intakeState = IntakeState.INTAKE_GRAB;
        }
    }

    private void intakeArmControl () {

        //Rising edge detector
        if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        }
        if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
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
