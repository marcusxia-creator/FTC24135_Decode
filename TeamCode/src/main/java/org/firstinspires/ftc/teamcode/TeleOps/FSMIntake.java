package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class FSMIntake {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    public INTAKESTATE intakeState = INTAKESTATE.INTAKE_START;
    private ElapsedTime debounceTimer = new ElapsedTime();

    public enum INTAKESTATE {
        INTAKE_START,
        INTAKE_FORWARD,
        INTAKE_REVERSE,
        INTAKE_STOP
    }
    public FSMIntake (RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }

    public void Init() {
        robot.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void IntakeLoop() {
        switch (intakeState){
            case INTAKE_START:
                if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
                    intakeState = INTAKESTATE.INTAKE_FORWARD;
                }
                if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
                    intakeState = INTAKESTATE.INTAKE_REVERSE;
                }
                break;
            case INTAKE_FORWARD:
                robot.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
                    intakeState = INTAKESTATE.INTAKE_STOP;
                }
                break;
            case INTAKE_REVERSE:
                robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
                    intakeState = INTAKESTATE.INTAKE_STOP;
                }
                break;
            case INTAKE_STOP:
                robot.intakeMotor.setPower(0);
                intakeState = INTAKESTATE.INTAKE_START;
                break;
            default:
                intakeState = INTAKESTATE.INTAKE_START;
        }
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

}



