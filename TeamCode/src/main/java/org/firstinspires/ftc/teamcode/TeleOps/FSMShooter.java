package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;



public class FSMShooter {
    private final GamepadEx gamepad_1;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    SHOOTERSTATE shooterState;
    public RAMPSTATE rampstate = RAMPSTATE.DOWN;
    int counter = 2;
    Spindexer spindexer;
    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

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
        FLYWHEEL_RUNNING,
        SHOOTING,
        SPINDEXER_ROTATE,
    }
    public enum RAMPSTATE{
        UP,
        DOWN
    }

    // Define slotAngle
    double [] slotAngle;

    GamepadManager gamepadManager;

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager) {
        this.gamepad_1 = gamepad_1;
        this.robot = robot;
        slotAngle = new double[]{spindexerSlot0, spindexerSlot1, spindexerSlot2};
        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
    }

    public void Init() {
        counter = 2;
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.leftGateServo.setPosition(gateDown);
        robot.rightGateServo.setPosition(gateDown);
        robot.spindexerServo.setPosition(slotAngle[counter]);
        shooterState = SHOOTERSTATE.IDLE;
        robot.shooterMotor.setPower(0);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void ShooterLoop() {
        // --- Global Controls (can be triggered from any state) ---
        // 'A' button is an emergency stop or reset.
        switch (shooterState) {
            case IDLE:
                robot.shooterMotor.setPower(0);
                robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                // Press 'X' to start spinning the flywheel
                if (gamepadManager.Flywheel.PressState) {
                    robot.shooterMotor.setPower(shooterSpeed);

                    //robot.shooterMotor.setPower(shooterPowerCalculator.getPower());
                    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                }
                if (gamepadManager.Purple.PressState) {
                    targetColour = Spindexer.SLOT.Purple;
                }
                if (gamepadManager.Green.PressState) {
                    targetColour = Spindexer.SLOT.Green;
                }
                break;
            case FLYWHEEL_RUNNING:
                // Give the flywheel time ProcessBuilder.Redirect.to get ProcessBuilder.Redirect.to speed (Log.e.g., 1 second)
                // Press 'Y' to toggle ramp up/down]
                if (gamepadManager.Launch.HoldState) {
                    shooterState = SHOOTERSTATE.SHOOTING;
                    rampstate = RAMPSTATE.UP;
                    updateServoState();
                    robot.leftGateServo.setPosition(gateUp);
                    robot.rightGateServo.setPosition(gateUp);
                    shootTimer.reset();
                }
                // Press 'X' again to stop spinning the flywheel
                // Allow driver to turn off flywheel if they change their mind with in 1 second
                if (gamepadManager.Flywheel.PressState) {
                    shooterState = SHOOTERSTATE.IDLE;
                    robot.shooterMotor.setPower(0);
                }

                //Launch Colour
                if ((gamepadManager.Purple.PressState)||!spindexer.checkFor(Spindexer.SLOT.Green)&&spindexer.checkFor(Spindexer.SLOT.Purple)) {
                    targetColour = Spindexer.SLOT.Purple;
                }
                if ((gamepadManager.Green.PressState)||!spindexer.checkFor(Spindexer.SLOT.Purple)&&spindexer.checkFor(Spindexer.SLOT.Green)) {
                    targetColour = Spindexer.SLOT.Green;
                }

                if (spindexer.slots[spindexer.currentSlot]!=targetColour){
                    if(spindexer.checkFor(targetColour)) {
                        spindexer.runToSlot(targetColour);
                        shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                    }
                    else{
                        shooterState = SHOOTERSTATE.IDLE;
                        robot.shooterMotor.setPower(0);
                    }
                }
                shootTimer.reset();
                break;
                
            case SHOOTING:
                //Ramp Down
                if (shootTimer.seconds() > 0.75 ) {
                    rampstate = RAMPSTATE.DOWN;
                    updateServoState();
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                }
                //Check Distance
                if(shootTimer.seconds() > 1.0){
                    double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
                    if (distance < 100) {
                        //Ramp Up
                        rampstate = RAMPSTATE.UP;
                        updateServoState();
                        robot.leftGateServo.setPosition(gateUp);
                        robot.rightGateServo.setPosition(gateUp);
                        shootTimer.reset();
                    } else {
                        //Countinue
                        spindexer.writeToCurrent(Spindexer.SLOT.Empty);
                        shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                        shootTimer.reset();
                    }
                }
                //Cancel
                if (gamepadManager.Flywheel.PressState) {
                    shooterState = SHOOTERSTATE.IDLE;
                    robot.shooterMotor.setPower(0);
                    rampstate = RAMPSTATE.DOWN;
                    updateServoState();
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                }
                break;

            case SPINDEXER_ROTATE:
                if (shootTimer.seconds() > 0.45) {
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                }
                break;

            default:
                robot.shooterMotor.setPower(0);
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
