package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Deprecated
public class FSMDepositControl {

    // Enums
    public enum LIFTSTATE {
        LIFT_START,            // Initial state
        LIFT_HIGHBASKET,       // Moving to high basket
        LIFT_SAMPLE_EXTEND,    // Extending sample
        LIFT_SAMPLE_DUMP,      // Dumping sample
        LIFT_RETRACT,          // Retracting lift
        COLOR_SAMPLE_DROP,     // Dropping color sample
        SPECIMEN_PICK,         // Picking specimen
        LIFT_HIGHBAR,          // High bar state
        LIFT_SPECIMEN_HOOK,    // Hooking specimen
        LIFT_SPECIMEN_SCORE    // Scoring specimen
    }

    public enum DEPOSITCLAWSTATE {
        OPEN,                  // Claw open
        CLOSE                  // Claw close
    }

    // Inner Classes
    public static class ColorRange {
        private final String name;
        private final int hueMin, hueMax;

        public ColorRange(String name, int hueMin, int hueMax) {
            this.name = name;
            this.hueMin = hueMin;
            this.hueMax = hueMax;
        }

        public String getName() { return name; }
        public int getHueMin() { return hueMin; }
        public int getHueMax() { return hueMax; }
    }

    // Constants

    // Member Variables
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineIntake intake;
    private final ElapsedTime liftTimer = new ElapsedTime();
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();

    private final List<ColorRange> colorRanges = new ArrayList<>();
    private LIFTSTATE liftState = LIFTSTATE.LIFT_START;
    private DEPOSITCLAWSTATE depositClawState = DEPOSITCLAWSTATE.OPEN;

    private float hue;
    private boolean empty;
    private static String detectedColor = "None";

    // Constructor
    public FSMDepositControl(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intake) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intake = intake;
        initializeColorRanges();
        runtime.reset();
    }

    // Initialization Method
    public void init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }

    // Main FSM Loop
    public void depositArmLoop() {
        detectColor();
        switch (liftState) {
            case LIFT_START:
                handleStartState();
                break;

            case LIFT_HIGHBASKET:
                handleLiftHighBasketState();
                break;

            case LIFT_SAMPLE_EXTEND:
                handleLiftSampleExtendState();
                break;

            case LIFT_SAMPLE_DUMP:
                handleLiftSampleDumpState();
                break;

            case LIFT_RETRACT:
                handleLiftRetractState();
                break;

            case COLOR_SAMPLE_DROP:
                handleColorSampleDropState();
                break;

            case SPECIMEN_PICK:
                handleSpecimenPickState();
                break;

            case LIFT_HIGHBAR:
                handleLiftHighBarState();
                break;

            case LIFT_SPECIMEN_HOOK:
                handleLiftSpecimenHookState();
                break;

            case LIFT_SPECIMEN_SCORE:
                handleLiftSpecimenScoreState();
                break;

            default:
                telemetry.addData("Error", "Unexpected state in liftState: " + liftState);
                liftState = LIFTSTATE.LIFT_START;
                break;
        }
        handleGlobalControls();
        handleGlobalClawControls();
    }

    // FSM State Handlers
    private void handleStartState() {
        if (isButtonPressedForHighBasket()) {
            liftState = LIFTSTATE.LIFT_HIGHBASKET;
            liftTimer.reset();
        } else if (isButtonPressedForSampleDrop()) {
            liftState = LIFTSTATE.COLOR_SAMPLE_DROP;
            liftTimer.reset();
        }
    }

    private void handleLiftHighBasketState() {
        if (liftTimer.milliseconds() > 100 && !detectedColor.equals("Black")) {
            prepareForSampleExtend();
            liftState = LIFTSTATE.LIFT_SAMPLE_EXTEND;
        } else {
            liftState = LIFTSTATE.LIFT_START;
        }
    }

    private void handleLiftSampleExtendState() {
        if (isLiftAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos)) {
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
            liftTimer.reset();
            liftState = LIFTSTATE.LIFT_SAMPLE_DUMP;
        }
    }

    private void handleLiftSampleDumpState() {
        if (liftTimer.seconds() >= RobotActionConfig.dumpTime) {
            depositClawState = DEPOSITCLAWSTATE.OPEN;
        }
        if (liftTimer.seconds() >= RobotActionConfig.postDumpTime) {
            liftState = LIFTSTATE.LIFT_RETRACT;
        }
    }

    private void handleLiftRetractState() {
        lifeRetract();
        if (isLiftAtDownPosition(RobotActionConfig.deposit_Slide_Down_Pos)) {
            robot.liftMotorLeft.setPower(0);
            robot.liftMotorRight.setPower(0);
            resetToTransferPosition();
            liftState = LIFTSTATE.LIFT_START;
        }
    }

    private void handleColorSampleDropState() {
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
        liftState = LIFTSTATE.SPECIMEN_PICK;
    }

    private void handleSpecimenPickState() {
        if (!empty && liftTimer.seconds() > RobotActionConfig.pickTime) {
            depositClawState = DEPOSITCLAWSTATE.CLOSE;
        }
        if (depositClawState == DEPOSITCLAWSTATE.CLOSE) {
            liftState = LIFTSTATE.LIFT_HIGHBAR;
        }
    }

    private void handleLiftHighBarState() {
        setLiftTarget(RobotActionConfig.deposit_Slide_Highbar_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
        if (isLiftAtPosition(RobotActionConfig.deposit_Slide_Highbar_Pos)) {
            liftState = LIFTSTATE.LIFT_SPECIMEN_HOOK;
        }
    }

    private void handleLiftSpecimenHookState() {
        if (depositClawState == DEPOSITCLAWSTATE.OPEN) {
            liftTimer.reset();
            liftState = LIFTSTATE.LIFT_SPECIMEN_SCORE;
        }
    }

    private void handleLiftSpecimenScoreState() {
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
        if (liftTimer.seconds() > 0.5) {
            driveBackward(10);
            liftState = LIFTSTATE.LIFT_RETRACT;
        }
    }

    // Global Controls
    private void handleGlobalControls() {
        if (isCancelButtonPressed()) {
            liftState = LIFTSTATE.LIFT_RETRACT;
        }
    }

    private void handleGlobalClawControls() {
        if (depositClawState != DEPOSITCLAWSTATE.OPEN) {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        } else {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }
    }

    // Utility Methods
    private void detectColor() {
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                RobotActionConfig.hsvValues);
        hue = RobotActionConfig.hsvValues[0];
        detectedColor = "None";
        empty = true;
        for (ColorRange range : colorRanges) {
            if (hue > range.getHueMin() && hue < range.getHueMax()) {
                detectedColor = range.getName();
                empty = false;
                break;
            }
        }
    }

    private boolean isButtonPressedForHighBasket() {
        return gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced() ;
    }

    private boolean isButtonPressedForSampleDrop() {
        return gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced();
    }

    private boolean isCancelButtonPressed() {
        return gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced();
    }

    private boolean isLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 &&
                Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }

    private boolean isLiftAtDownPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 15 &&
                Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 15;
    }

    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    private void prepareForSampleExtend(){
        setLiftTarget(RobotActionConfig.deposit_Slide_Highbasket_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
    }

    private void lifeRetract(){
        setLiftTarget(RobotActionConfig.deposit_Slide_Down_Pos,RobotActionConfig.deposit_Slide_DownLiftPower);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
    }
    private void setLiftTarget(int targetPosition, double power) {
        robot.liftMotorLeft.setTargetPosition(targetPosition);
        robot.liftMotorRight.setTargetPosition(targetPosition);
        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);
    }

    private void resetToTransferPosition() {
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
    }

    private void initializeColorRanges() {
        colorRanges.add(new ColorRange("Black", 155, 170));
        colorRanges.add(new ColorRange("Red", 15, 25));
        colorRanges.add(new ColorRange("Blue", 220, 230));
        colorRanges.add(new ColorRange("Yellow", 70, 80));
    }

    private void driveBackward(double distanceCm) {
        double targetPosition = distanceCm * RobotActionConfig.TICKS_PER_CM;
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - (int) targetPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - (int) targetPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - (int) targetPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - (int) targetPosition);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftMotor.setPower(0.5);
        robot.frontRightMotor.setPower(0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(0.5);

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            telemetry.addData("Driving", "Backward");
            telemetry.update();
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetDepositClawState(DEPOSITCLAWSTATE state) {
        this.depositClawState = state;
    }

    LIFTSTATE returnLiftstate(){
        return liftState;
    }
    DEPOSITCLAWSTATE returnDepositClawState(){
        return depositClawState;
    }
}
