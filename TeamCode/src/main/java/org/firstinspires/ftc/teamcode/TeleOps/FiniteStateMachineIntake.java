package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Button Config for intake
 *Start state
 * *DPad Right          : retract/extend

 *Pick state
 * *Dpad up             : rise intake arm
 * *Dpad down           : lower intake arm
 * *left bumper         : to rotate left
 * *right bumper        : to rotate right

 * universal state
 * *Right Trigger + DPADRIGHT : Lower the intake arm only
 * *A                   : to open/close intake

 * Action for intake
 * *default open intake when extend
 * *default close intake when retract
 */
public class FiniteStateMachineIntake {

    //Intake STATE
    public enum INTAKESTATE {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_PICK,
        INTAKE_RETRACT,
        INTAKE_TRANS
    }

    // Robot and Gamepad Member
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineDeposit depositArmDrive;
    //private final FSMDepositControl depositArmDrive;

    //Time member
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    //Intake states
    public INTAKESTATE intakeState; // Persisting state
    private ElapsedTime intakeTimer = new ElapsedTime(); // Timer for controlling dumping time
    public INTAKECLAWSTATE intakeClawState; //claw default open

    private double intakeArmPosition;
    private double rotationPosition;
    FiniteStateMachineDeposit.LIFTSTATE depositArmState;

    //Constructor
    public FiniteStateMachineIntake(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineDeposit depositArmDrive) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.depositArmDrive = depositArmDrive;
        this.intakeClawState = INTAKECLAWSTATE.OPEN;
        this.intakeState = INTAKESTATE.INTAKE_START;
    }


    //Initialization
    public void Init() {
        intakeTimer.reset();
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
    }

    //FSM Loop Control
    public void IntakeArmLoop() {
        // Display current lift state and telemetry feedback
        switch (intakeState) {
            case INTAKE_START:
                /** Debounce the button press 'DPAD_RIGHT' for starting the lift extend */
                if (((gamepad_1.getButton(DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced()) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    robot.intakeClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.OPEN);
                    intakeClawState = INTAKECLAWSTATE.OPEN;
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_EXTEND;
                }
                break;
            case INTAKE_EXTEND:
                // after 0.5s intake arm lower for pick up
                if (intakeTimer.seconds() > RobotActionConfig.intakeSlideExtendTime) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_PICK;
                }
            case INTAKE_PICK:
                rotationPosition = robot.intakeRotationServo.getPosition();
                intakeArmPosition = (robot.intakeRightArmServo.getPosition());
                /** claw rotation - LEFT BUMPER */
                if ((gamepad_1.getButton(LEFT_BUMPER) || gamepad_2.getButton(LEFT_BUMPER)) && isButtonDebounced()) {
                    //use to be 0.01
                    rotationPosition += 0.15;
                    robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                }
                /** claw rotation RIGHT BUMPER */
                if ((gamepad_1.getButton(RIGHT_BUMPER) || gamepad_2.getButton(RIGHT_BUMPER)) && (isButtonDebounced())) {
                    //use to be 0.01
                    rotationPosition -= 0.15;
                    robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                }

                /** intake arm up DPAD UP */
                if ((gamepad_1.getButton(DPAD_UP) || gamepad_2.getButton(DPAD_UP)) && isButtonDebounced()) {
                    //use to be 0.01
                    intakeArmPosition -= 0.05;
                    robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                    robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                }

                /** intake arm down DPAD DOWN */
                if ((gamepad_1.getButton(DPAD_DOWN) || gamepad_2.getButton(DPAD_DOWN)) && isButtonDebounced()) {
                    //use to be 0.01
                    intakeArmPosition += 0.05;
                    robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                    robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                }

                /** intake retract
                 * DPAD_RIGHT again
                 */
                if ((gamepad_1.getButton(DPAD_RIGHT) || gamepad_2.getButton(DPAD_RIGHT)) && isButtonDebounced()) {
                    intakeClawState = INTAKECLAWSTATE.CLOSE;
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                    //retract
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_RETRACT;
                }

                break;

            case INTAKE_RETRACT:
                // Wait for the pickup time to pass
                if (intakeTimer.seconds() > RobotActionConfig.waitTime) {
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    //robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);        // set wrist to transfer position
                }
                if (intakeTimer.seconds() > RobotActionConfig.waitTime + RobotActionConfig.intakeWristRotationTime) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_slide_Retract_Set);// set slide retract 2/3
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_TRANS;
                }
                break;

            case INTAKE_TRANS:
                // read in deposit arm state and set deposit arm state
                // set deposit claw state -- open
                depositArmState = depositArmDrive.liftState;
                depositArmDrive.SetDepositstate(FiniteStateMachineDeposit.LIFTSTATE.LIFT_START);
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.OPEN);

                // Check if the intakeslide has reached the position
                if (intakeTimer.seconds() > RobotActionConfig.intakeSlideExtendTime*0.4) {              // wait 0.5 second for slide retract 2/3
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);        // set intake arm  to transfer
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);       // set intake arm to transfer
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                }
                if (intakeTimer.seconds() > RobotActionConfig.intakeSlideExtendTime) {                  // wait another  0.5second to retract slide again.
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);     // set slide retract
                }
                if (depositArmState == FiniteStateMachineDeposit.LIFTSTATE.LIFT_START && intakeTimer.seconds() >= RobotActionConfig.transferTime + RobotActionConfig.intakeSlideExtendTime) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.CLOSE);
                }
                if (intakeTimer.seconds() >= RobotActionConfig.transferTime + RobotActionConfig.intakeSlideExtendTime+RobotActionConfig.waitTime) {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    intakeClawState = INTAKECLAWSTATE.OPEN;
                    intakeState = INTAKESTATE.INTAKE_START;
                }
                break;

        }

        /** Lower intake arm only for grabbing - Right Trigger + DPAD_RIGHT */
        if ((gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ||
                (gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) &&
                        isButtonDebounced()) {
            debounceTimer.reset();
            intakeClawState = INTAKECLAWSTATE.OPEN;
            IntakeClawSwitch();
            intakeState = INTAKESTATE.INTAKE_EXTEND;
        }

        /** Claw control - Button A */
        // add in the button "A" for intake claw open and close
        if (((gamepad_1.getButton(GamepadKeys.Button.A) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                (gamepad_2.getButton(GamepadKeys.Button.A) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                isButtonDebounced()) {
            ToggleClaw();
            IntakeClawSwitch();
        }
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean IsSlideAtPosition(double targetPosition) {
        return Math.abs(robot.intakeLeftSlideServo.getPosition() - targetPosition) < 0.005;
    }

    /**
     * Claw control
     * Claw state
     * Claw Toggle
     * Claw open / close
     */
    //Claw State
    public enum INTAKECLAWSTATE {
        OPEN,
        CLOSE
    }

    // set claw state
    public void SetInTakeClawState(INTAKECLAWSTATE state) {
        this.intakeClawState = state;
    }

    //Toggle Claw()
    private void ToggleClaw() {
        if (intakeClawState == INTAKECLAWSTATE.OPEN) {
            intakeClawState = INTAKECLAWSTATE.CLOSE;
        } else {
            intakeClawState = INTAKECLAWSTATE.OPEN;
        }
    }

    private void IntakeClawSwitch() {
        if (intakeClawState == INTAKECLAWSTATE.OPEN) {
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        } else {
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
        }
    }

    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}
