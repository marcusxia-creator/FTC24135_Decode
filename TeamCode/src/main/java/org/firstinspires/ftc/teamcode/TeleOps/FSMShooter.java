package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;



public class FSMShooter {
    private final GamepadEx gamepad_1;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    private SHOOTERSTATE shooterState;
    public RAMPSTATE rampstate = RAMPSTATE.DOWN;
    private int counter = 2;

    /**
     * BUTTON FOR SHOOTING
     * * Button A is global KEY, --- Global Controls (can be triggered from any state) ---
     *   cancel the actionS & Exit the loop for this cycle
     * * Button X/Square is local key, --- IDLE STATE---
     *   Press 'X/Square' to start spinning the flywheel
     * * Button X/Square is local key, --- FLYWHEEL STATE---
     *   Cancle the FLYWHEEL within 1 second
     * * Button Y/Triangle is local key, --- RAMPUP STATE---
     *   Press 'Y/Triangle' to toggle ramp up to launch ball to shooter
     */


    public enum SHOOTERSTATE {
        IDLE,
        FLYWHEEL_START,
        RAMP_UP,
        DISTANCE_SENSOR_CHECK,
        SPINDEXER_ROTATE,
        COOLDOWN
    }
    public enum RAMPSTATE{
        UP,
        DOWN
    }

    // Define slotAngle
    double [] slotAngle;

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot) {
        this.gamepad_1 = gamepad_1;
        this.robot = robot;
        slotAngle = new double[]{spindexerSlot0, spindexerSlot1, spindexerSlot2};
    }

    public void Init() {
        counter = 2;
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.leftGateServo.setPosition(gateDown);
        robot.rightGateServo.setPosition(gateDown);
        robot.spindexerServo.setPosition(slotAngle[counter]);
        shooterState = SHOOTERSTATE.IDLE;;
    }

    public void ShooterLoop() {
        // --- Global Controls (can be triggered from any state) ---
        // 'A' button is an emergency stop or reset.
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            Init();
            return; // Exit the loop for this cycle
        }
        switch (shooterState) {
            case IDLE:
                // Press 'X' to start spinning the flywheel
                if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
                    robot.shooterMotor.setPower(shooterSpeed);
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.FLYWHEEL_START;
                }
                break;
            case FLYWHEEL_START:
                // Give the flywheel time ProcessBuilder.Redirect.to get ProcessBuilder.Redirect.to speed (Log.e.g., 1 second)
                if (shootTimer.seconds() > 1.0) {
                    shooterState = SHOOTERSTATE.RAMP_UP;
                }
                // Press 'X' again to stop spinning the flywheel
                // Allow driver to turn off flywheel if they change their mind with in 1 second
                if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
                    shooterState = SHOOTERSTATE.COOLDOWN;
                }
                break;
            case RAMP_UP:
                // Press 'Y' to toggle ramp up/down
                if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
                    rampstate = RAMPSTATE.UP;
                    updateServoState();
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.DISTANCE_SENSOR_CHECK;
                }
                break;
            case DISTANCE_SENSOR_CHECK:
                if (shootTimer.seconds() > 0.5 ) {
                        rampstate = RAMPSTATE.DOWN;
                        updateServoState();
                }
                if(shootTimer.seconds() > 1.0){
                    double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
                    if (distance < 100) {
                        shooterState = SHOOTERSTATE.RAMP_UP;
                        shootTimer.reset();
                    } else {
                        shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                        shootTimer.reset();
                    }
                }
                break;
            case SPINDEXER_ROTATE:
                // Decrement counter to aim at the next slot
                counter--;
                if (counter >= 0) {
                    robot.spindexerServo.setPosition(slotAngle[counter]);
                    if (shootTimer.seconds() > 0.45) {
                        shooterState = SHOOTERSTATE.RAMP_UP;
                    }
                } else {
                    shooterState = SHOOTERSTATE.COOLDOWN;
                }
                shootTimer.reset();
                break;
            case COOLDOWN:
                // Spin down the motor and reset state to IDLE
                robot.shooterMotor.setPower(0);
                // After cooling down, we go back to the beginning.
                // Re-initialize to reset the counter for the next cycle.
                Init();
                shooterState = SHOOTERSTATE.IDLE;
                break;
            default:
                shooterState = SHOOTERSTATE.IDLE;
        }
    }

    private void toggleRamp(){
        if (rampstate == RAMPSTATE.UP){
            rampstate = RAMPSTATE.DOWN;
        }else{
            rampstate = RAMPSTATE.UP;
        }
    }
    private void updateServoState (){
        if (rampstate != RAMPSTATE.UP){
            robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        } else{
            robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
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

