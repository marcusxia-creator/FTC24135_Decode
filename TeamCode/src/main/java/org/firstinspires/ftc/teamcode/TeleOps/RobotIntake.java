package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
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

    private final double intakeSlideExtendedPosition;
    private final double intakeSlideRetractedPosition;
    private final double intakeArmExtendedPosition;
    private final double intakeArmRetractedPosition;
    private final double intakeRotationServoNeutralPosition;
    private final double intakeClawServoClosePosition;
    private final double intakeClawServoOpenPosition;
    private final double intakeArmIdlePosition;

    private final double intakeArmServoChangeAmount;
    private final double intakeRotationServoSteerAmount;

    private final double depositServoClawClosePosition;

    //Set up timer for debouncing
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD;

    private final ElapsedTime intakeTimer = new ElapsedTime();

    //Constructor
    public RobotIntake
        (
            Gamepad gamepad1, Gamepad gamepad2, RobotHardware robot,

            double intakeSlideExtendedPosition, double intakeSlideRetractedPosition,
            double intakeArmExtendedPosition, double intakeArmRetractedPosition,
            double intakeRotationNeutralPosition,
            double intakeClawServoClosePosition,double intakeClawServoOpenPosition,

            double intakeArmServoChangeAmount, double intakeRotationServoSteerAmount,

            double intakeArmIdlePosition,

            double depositServoClawClosePosition,

            double debounce_threshold
        ) {

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;

        this.intakeSlideExtendedPosition = intakeSlideExtendedPosition;
        this.intakeSlideRetractedPosition = intakeSlideRetractedPosition;
        this.intakeArmExtendedPosition = intakeArmExtendedPosition;
        this.intakeArmRetractedPosition = intakeArmRetractedPosition;
        this.intakeRotationServoNeutralPosition = intakeRotationNeutralPosition;
        this.intakeClawServoClosePosition = intakeClawServoClosePosition;
        this.intakeClawServoOpenPosition = intakeClawServoOpenPosition;

        this.intakeArmServoChangeAmount = intakeArmServoChangeAmount;
        this.intakeRotationServoSteerAmount = intakeRotationServoSteerAmount;

        this.intakeArmIdlePosition = intakeArmIdlePosition;

        this.depositServoClawClosePosition = depositServoClawClosePosition;

        this.DEBOUNCE_THRESHOLD = debounce_threshold;

        this.intakeState = IntakeState.INTAKE_EXTEND;
    }

    public void intakeSlideControl () {
        switch (intakeState) {
            case INTAKE_EXTEND:
                if ((gamepad1.dpad_right || gamepad2.dpad_right) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeLeftArmServo.setPosition(intakeArmExtendedPosition);
                    robot.intakeRightArmServo.setPosition(intakeArmExtendedPosition);
                    robot.intakeSlideServo.setPosition(intakeSlideExtendedPosition);
                    intakeState = IntakeState.INTAKE_GRAB;
                }
                else {
                    intakeInit(intakeSlideRetractedPosition, intakeArmRetractedPosition, intakeRotationServoNeutralPosition, intakeClawServoOpenPosition);
                }
                break;
            case INTAKE_GRAB:
                if ((gamepad1.dpad_left || gamepad2.dpad_left) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeClawServo.setPosition(intakeClawServoClosePosition);
                    robot.intakeRotationServo.setPosition(intakeRotationServoNeutralPosition);
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                else {
                    intakeArmControl(intakeArmServoChangeAmount);
                    intakeRotationServoSteer(intakeRotationServoSteerAmount);
                }
                break;
            case INTAKE_RETRACT:
                robot.intakeSlideServo.setPosition(intakeSlideRetractedPosition);
                if (intakeTimer.seconds() > 0.5) {
                    intakeTimer.reset();
                    robot.intakeLeftArmServo.setPosition(intakeArmRetractedPosition);
                    robot.intakeRightArmServo.setPosition(intakeArmRetractedPosition);
                    intakeState = IntakeState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER:
                if(intakeTimer.seconds() > 1) {
                    robot.intakeClawServo.setPosition(intakeClawServoOpenPosition);
                }
                if(intakeTimer.seconds() > 1.3) {
                    robot.depositClawServo.setPosition(depositServoClawClosePosition);
                }
                if(intakeTimer.seconds() > 1.5) {
                    robot.intakeLeftArmServo.setPosition(intakeArmIdlePosition);
                    robot.intakeRightArmServo.setPosition(intakeArmIdlePosition);
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

    private void intakeInit (double intakeSlideRetractedPosition, double intakeArmRetractedPosition, double intakeRotationServoNeutralPosition, double intakeClawServoOpenPosition) {
        robot.intakeSlideServo.setPosition(intakeSlideRetractedPosition);
        robot.intakeLeftArmServo.setPosition(intakeArmRetractedPosition);
        robot.intakeRightArmServo.setPosition(intakeArmRetractedPosition);
        robot.intakeRotationServo.setPosition(intakeRotationServoNeutralPosition);
        robot.intakeClawServo.setPosition(intakeClawServoOpenPosition);
    }

    private void intakeArmControl (double armServoChangeAmount) {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() + armServoChangeAmount);
        }
        if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() - armServoChangeAmount);
        }
    }

    private void intakeRotationServoSteer(double intakeRotationServoSteer) {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        //Rising edge detector
        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() + intakeRotationServoSteer);
        }
        if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) || (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() - intakeRotationServoSteer);
        }
    }
}
