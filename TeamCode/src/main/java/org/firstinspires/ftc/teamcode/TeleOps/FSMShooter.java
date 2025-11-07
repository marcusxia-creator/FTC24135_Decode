package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;



public class FSMShooter {
    public ShooterPowerCalculator shooterPowerCalculator;
    private final GamepadEx gamepad_1;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    SHOOTERSTATE shooterState;
    public RAMPSTATE rampstate = RAMPSTATE.DOWN;
    Spindexer spindexer;
    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

    /**
     * BUTTON FOR SHOOTING
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

    public Spindexer.Motif motif;

    GamepadManager gamepadManager;

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager, ShooterPowerCalculator shooterPowerCalculator) {
        this.gamepad_1 = gamepad_1;
        this.robot = robot;
        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
        this.shooterPowerCalculator = shooterPowerCalculator;
    }

    public void Init() {
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.leftGateServo.setPosition(gateDown);
        robot.rightGateServo.setPosition(gateDown);
        shooterState = SHOOTERSTATE.IDLE;
        robot.shooterMotor.setPower(0);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooterPowerCalculator = new ShooterPowerCalculator();
        //shooterPowerCalculator.init(robot);
        motif = spindexer.GPP;//Temporary
    }

    public void ShooterLoop() {
        double voltage = robot.getBatteryVoltageRobust();
        double speed = shooterPowerCalculator.getPower();
        double power_setpoint = speed*12.0/ voltage;
        // --- Global Controls (can be triggered from any state) ---
        // 'A' button is an emergency stop or reset.
        switch (shooterState) {
            case IDLE:
                robot.shooterMotor.setPower(0);
                robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                /**
                // Find dI
                I=robot.shooterMotor.getCurrent(CurrentUnit.AMPS);
                dt=(I-lastI)/0.00091;
                lastI=I;
                 **/
                // Press 'X' to start spinning the flywheel
                if (gamepadManager.Flywheel.PressState) {
                    //robot.shooterMotor.setVelocity(shooterVel);
                    robot.shooterMotor.setPower(Range.clip(power_setpoint,0.3,1.0));

                    //robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced() && (robot.shooterMotor.getPower() >= (power_setpoint - 0.008) && robot.shooterMotor.getPower() <= (power_setpoint + 0.01)) /*&& shooterVel*shooterFactorThreshold<=robot.shooterMotor.getVelocity()*/) {
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

                if (gamepadManager.autoMotif.ToggleState && spindexer.checkMotif(motif)){
                    targetColour=spindexer.motifColour(motif);
                }
                else {
                    //Launch Colour
                    if ((gamepadManager.Purple.PressState) || !spindexer.checkFor(Spindexer.SLOT.Green) && spindexer.checkFor(Spindexer.SLOT.Purple)) {
                        targetColour = Spindexer.SLOT.Purple;
                    }
                    if ((gamepadManager.Green.PressState) || !spindexer.checkFor(Spindexer.SLOT.Purple) && spindexer.checkFor(Spindexer.SLOT.Green)) {
                        targetColour = Spindexer.SLOT.Green;
                    }
                }

                if (spindexer.slotColour()!=targetColour){
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
