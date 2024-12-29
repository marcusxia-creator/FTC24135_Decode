package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

/** Button Config for deposit
 * *X                   : extend
 * *B                   : Cancel
 */

public class RobotDeposit {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    public enum LIFTSTATE {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    private DEPOSITSTATE depositState;

    private LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2;// Debouncing threshold for button presses
    private ElapsedTime hangTimer = new ElapsedTime();
    private final double HANG_TIME = 1;
    private ElapsedTime extendTimer = new ElapsedTime();
    private final double EXTENDTIME = 1;

    public RobotDeposit (GamepadEx gamepad_1,
                                 GamepadEx gamepad_2, RobotHardware robot
                         ) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }

    // Deposit Arm Control
    public void depositArmLoop() {
        // Display current lift state and telemetry feedback
        switch (liftState) {
            case LIFT_START:
                // Debounce the button press for starting the lift extend
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && (gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.2))
                        || (gamepad_2.getButton(GamepadKeys.Button.X) && (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.2))) && RobotActionConfig.depositState == RobotActionConfig.DepositState.DEPOSIT_EXTEND && debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    liftState = LIFTSTATE.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (isLiftAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos)) {
                    //move deposit arm to dump
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_dump_Pos);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_dump_Pos);
                    // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_dump_Pos);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                }
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime+0.5) {
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);// Reset servo to idle
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_retract_Pos);
                    if (liftTimer.seconds() >= RobotActionConfig.dumpTime +1.5) {
                        liftState = LIFTSTATE.LIFT_RETRACT;
                    }
                }
                break;
            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if(servo_AtPosition(RobotActionConfig.deposit_Claw_Open) && liftTimer.seconds()>=RobotActionConfig.retractTime) {
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos); // Start retracting the lift
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                }
                if (isLiftAtPosition(RobotActionConfig.deposit_Slide_down_Pos)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
                    RobotActionConfig.depositState = RobotActionConfig.DepositState.DEPOSIT_IDLE;
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;
            default:
                liftState = LIFTSTATE.LIFT_START;
                break;
        }

        // Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && liftState != LIFTSTATE.LIFT_START) {
            debounceTimer.reset();
            liftState = LIFTSTATE.LIFT_START;
            robot.liftMotorLeft.setPower(0); // Ensure the motor is stopped
            robot.liftMotorRight.setPower(0);
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_retract_Pos);
            robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);
            robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }

        // Claw control - Button Back
        if((gamepad_1.getButton(GamepadKeys.Button.Y) || gamepad_2.getButton(GamepadKeys.Button.Y))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            ToggleDeposit();
            if (depositState == DEPOSITSTATE.OPEN) {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
            } else {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
            }
        }

        //
        if (hangTimer.seconds()> HANG_TIME){
            if ((gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7 && gamepad_1.getButton(GamepadKeys.Button.X)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD || (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7 && gamepad_2.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD)){
                debounceTimer.reset();
                extendTimer.reset();
                robot.intakeRightArmServo.setPosition(0.2);
                robot.intakeLeftArmServo.setPosition(0.2);
                robot.intakeSlideServo.setPosition(0.4);
                while (extendTimer.seconds() < 1.0){
                }
                robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hang);
                robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Hang);
                robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hang);
            }

        }
    }

    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        hangTimer.reset();
        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Initial_Pos);
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_retract_Pos);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }

    private boolean servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    LIFTSTATE State(){
        return liftState;
    }

    //Deposit Claw State
    public enum DEPOSITSTATE {
        OPEN,
        CLOSE
    }

    //Toggle Deposit Open - Close
    private void ToggleDeposit() {
        if (depositState == DEPOSITSTATE.OPEN) {
            depositState = DEPOSITSTATE.CLOSE;
        } else {
            depositState = DEPOSITSTATE.OPEN;
        }
    }
}