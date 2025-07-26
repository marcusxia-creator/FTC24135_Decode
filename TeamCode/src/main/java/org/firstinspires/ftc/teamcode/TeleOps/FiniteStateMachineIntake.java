package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.robot.Robot;
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

        /** For high basket */
        INTAKE_TRANS_READY,
        INTAKE_TRANS,

        /** For specimens */
        INTAKE_SPECIMEN_RETRACT,
        INTAKE_DROP_OFF,
        INTAKE_COLOR_SAMPLE_DROP
    }

    // Robot and Gamepad Member
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineDeposit depositArmDrive;
    //private final FSMDepositControl depositArmDrive;

    //Time member
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private ElapsedTime intakeTransTimer = new ElapsedTime();

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
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract_Initial);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Initial);
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);

    }

    //FSM Loop Control
    public void IntakeArmLoop() {
        // Display current lift state and telemetry feedback
        switch (intakeState) {
            case INTAKE_START:
                /**
                 robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Idle);
                 robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Idle);
                 robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                 */
                /** Debounce the button press 'DPAD_RIGHT' for starting the lift extend */
                if ((((gamepad_1.getButton(DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))

                        || (gamepad_1.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced()) {
                    // rotate intake arm to idle
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                    if (robot.intakeWristServo.getPosition() != RobotActionConfig.intake_Wrist_Grab){
                        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                    }
                    // reset time for next step
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_EXTEND;
                }
                break;
            case INTAKE_EXTEND:
                robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                intakeClawState = INTAKECLAWSTATE.OPEN;
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                if(intakeTimer.seconds()>RobotActionConfig.intakeWristRotationTime) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                    if (robot.intakeWristServo.getPosition() != RobotActionConfig.intake_Wrist_Grab){
                        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);}
                }
                if(intakeTimer.seconds()>(RobotActionConfig.intakeWristRotationTime+RobotActionConfig.intakeSlideExtendTime)) {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_PICK;
                }
                break;
            case INTAKE_PICK:
                if (intakeTimer.seconds() > RobotActionConfig.intakeSlideExtendTime) {
                    rotationPosition = robot.intakeRotationServo.getPosition();
                    intakeArmPosition = robot.intakeArmServo.getPosition();
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
                        robot.intakeArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                    }

                    /** intake arm down DPAD DOWN */
                    if ((gamepad_1.getButton(DPAD_DOWN) || gamepad_2.getButton(DPAD_DOWN)) && isButtonDebounced()) {
                        //use to be 0.01
                        intakeArmPosition += 0.05;
                        robot.intakeArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.58));
                    }

                    /** intake retract for sample transfer
                     * DPAD_RIGHT again
                     */

                    if ((gamepad_1.getButton(DPAD_RIGHT) || gamepad_2.getButton(DPAD_RIGHT)) && isButtonDebounced()) {
                        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                        //retract
                        intakeTimer.reset();
                        intakeState = INTAKESTATE.INTAKE_TRANS_READY;
                    }

                    /** intake retract for specimen transfer
                     * DPAD_LEFT again
                     */

                    if ((gamepad_1.getButton(DPAD_LEFT) || gamepad_2.getButton(DPAD_LEFT)) && isButtonDebounced()) {
                        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                        //retract
                        intakeTimer.reset();
                        intakeState = INTAKESTATE.INTAKE_SPECIMEN_RETRACT;
                    }
                }
                break;

            /** For high basket picking **/
            case INTAKE_TRANS_READY:
                // Wait for the pickup time to pass
                if (intakeTimer.seconds() > RobotActionConfig.waitTime) {
                    intakeClawState = INTAKECLAWSTATE.CLOSE;    /// Intake claw is controlled by state, state drive the open/close by IntakeClawSwitch() helper
                    IntakeClawSwitch();
                }
                if (intakeTimer.seconds() > RobotActionConfig.waitTime * 3.5) {
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);// wait 0.5 second for slide retract 2/3
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_TRANS;
                }
                break;

            case INTAKE_TRANS:
                // read in deposit arm state and set deposit arm state
                // set deposit claw state -- open
                depositArmState = depositArmDrive.liftState;
                depositArmDrive.SetDepositstate(FiniteStateMachineDeposit.LIFTSTATE.LIFT_START);
                depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.OPEN);
                if(intakeTransTimer.seconds() > RobotActionConfig.waitTime) {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);        // set intake arm to transfer;
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);      // set intake Wrist to transfer;
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                }
                if (intakeTransTimer.seconds() >  RobotActionConfig.waitTime*2) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);     // set slide retract
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                }

                if (depositArmState == FiniteStateMachineDeposit.LIFTSTATE.LIFT_START &&
                        intakeTimer.seconds() > RobotActionConfig.transferTime) {
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.CLOSE);
                }
                if (intakeTimer.seconds() > RobotActionConfig.transferTime + 0.2) {
                    intakeClawState = INTAKECLAWSTATE.OPEN;
                    IntakeClawSwitch();
                    intakeState = INTAKESTATE.INTAKE_START;
                }
                break;

            /** For specimen picking
             * from INTAKE_PICK state - After DPAD_RIGHT Button*/
            case INTAKE_SPECIMEN_RETRACT:
                if (intakeTimer.seconds() > RobotActionConfig.waitTime) {
                    intakeClawState = INTAKECLAWSTATE.CLOSE;
                    IntakeClawSwitch();
                }
                if (intakeTimer.seconds() > RobotActionConfig.waitTime*3.5) {
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                }

                if (intakeTimer.seconds() > RobotActionConfig.waitTime*3) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    intakeState = INTAKESTATE.INTAKE_DROP_OFF;
                }
                break;

            case INTAKE_DROP_OFF:
                if ((gamepad_1.getButton(DPAD_LEFT) || gamepad_2.getButton(DPAD_LEFT)) && isButtonDebounced()) {
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Side_Drop);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Side_Drop);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Side_Drop);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Side_Drop);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_COLOR_SAMPLE_DROP;
                }
                if ((gamepad_1.getButton(DPAD_RIGHT) || gamepad_2.getButton(DPAD_RIGHT)) && isButtonDebounced()) {
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_TRANS;
                }
                break;
            case INTAKE_COLOR_SAMPLE_DROP:
                if (intakeTimer.seconds() > RobotActionConfig.intakeTurretTurnTime+RobotActionConfig.waitTime) {
                    intakeClawState = INTAKECLAWSTATE.OPEN;
                    IntakeClawSwitch();
                }
                if (intakeTimer.seconds() > RobotActionConfig.intakeTurretTurnTime + RobotActionConfig.waitTime*3) {
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
                    /** this is the optimum position for intake arm*/
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    intakeState = INTAKESTATE.INTAKE_START;
                }
                break;
            default:
                intakeState = INTAKESTATE.INTAKE_START;
                break;
        }

        /** Lower intake arm only for grabbing - Right Trigger + DPAD_RIGHT */
        if ((gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ||
                (gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) &&
                        isButtonDebounced()) {
            debounceTimer.reset();
            intakeClawState = INTAKECLAWSTATE.OPEN;
            IntakeClawSwitch();
            robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
            robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
            intakeState = INTAKESTATE.INTAKE_TRANS_READY;
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
    ///Claw State
    public enum INTAKECLAWSTATE {
        OPEN,
        CLOSE
    }

    /// set claw state
    public void SetInTakeClawState(INTAKECLAWSTATE state) {
        this.intakeClawState = state;
    }

    ///Toggle Claw()
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

    /// Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}