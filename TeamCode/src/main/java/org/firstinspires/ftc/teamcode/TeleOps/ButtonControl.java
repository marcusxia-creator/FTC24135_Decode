package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
public class ButtonControl {
    private final GamepadInput gamepadInput;
    private final BasicTeleOp_RED_ALLIANCE redMainLoop;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private ElapsedTime debounceTimer = new ElapsedTime();
    // add constructor for blueMainLoop
    BasicTeleOp_RED_ALLIANCE.RobotActionState actionStates = BasicTeleOp_RED_ALLIANCE.RobotActionState.Idle;
    public ButtonControl(GamepadInput gamepadInput, BasicTeleOp_RED_ALLIANCE redMainLoop, GamepadEx gamepad1, GamepadEx gamepad2) {
        this.gamepadInput = gamepadInput;
        this.redMainLoop = redMainLoop;
        gamepad_1 = gamepad1;
        gamepad_2 = gamepad2;
    }
    public void control(){
        actionStates =  BasicTeleOp_RED_ALLIANCE.RobotActionState.Idle;
        //For sequence shooting
        if (gamepad_1.getButton(GamepadKeys.Button.X) || gamepad_2.getButton(GamepadKeys.Button.X)
                && isButtonDebounced()){
            actionStates = BasicTeleOp_RED_ALLIANCE.RobotActionState.Sequence_Shooting;
        }
        //For sort shooting
        if (gamepad_1.getButton(GamepadKeys.Button.Y) || gamepad_2.getButton(GamepadKeys.Button.Y)
                && isButtonDebounced()){
            actionStates = BasicTeleOp_RED_ALLIANCE.RobotActionState.Sort_Shooting;
        }
        //For intaking
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT)
                && isButtonDebounced()){
            actionStates = BasicTeleOp_RED_ALLIANCE.RobotActionState.Intaking;
        }
        //For reversing intake
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) || gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)
                && isButtonDebounced()) {
            redMainLoop.FSMIntake.reversing();

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
