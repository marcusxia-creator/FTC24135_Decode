package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;


/** Button Config for deposit
 * *X                           : high basket extend State          - LOCAL STATE - LIFT_START
 * * XX                         : red and blue high basket
 * * XY                         : red and blue sample drop
 * *Y                           : Specimen score                   - LOCAL STATE - LIFT_START
 * *B                           : Cancel;back to transfer pos       - GLOBAL STATE
 * *A                           : TOGGLE DEPOSIT CLAW OPEN/CLOSE    - GLOBAL STATE
 * *DPAD UP && LEFT BUMPER      : Hung                              - GLOBAL STATE -ACTIVE AFTER 100S
 * *DPAD DOWN && LEFT BUMPER    : Hung DOWN - 300 TICK EVERY PRESS  - GLOBAL STATE -ACTIVE AFTER 100S
 */

public class FiniteStateMachineDeposit {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    //bring in the finitemachinestateintake
    private final FiniteStateMachineIntake intake;

    /**
     * Deposit Arm State
     */
    public enum LIFTSTATE {
        LIFT_START,
        LIFT_SAMPLE_BRANCH,
        LIFT_SPECIMEN_BRANCH,
        LIFT_HIGHBASKET,
        LIFT_SAMPLE_EXTEND,
        LIFT_SAMPLE_DUMP,
        LIFT_RETRACT,
        COLOR_SAMPLE_DROP,
        SPECIMEN_PICK,
        LIFT_HIGHBAR,
        LIFT_SPECIMEN_HOOK,
        LIFT_SPECIMEN_SCORE,
    }

    /**
     * Deposit Claw State
     */
    public enum DEPOSITCLAWSTATE {
        OPEN,
        CLOSE
    }

    /**
     * ColorRange class used as an object for color threshold variable
     * it is similar to dictionary
     */
    class ColorRange {
        public String colorName;
        public int hueMin, hueMax;

        public ColorRange(String colorName, int hueMin, int hueMax) {
            this.colorName = colorName;
            this.hueMin = hueMin;
            this.hueMax = hueMax;
        }
    }

    /**
     * member DECLEAR
     */
    // STATE
    public DEPOSITCLAWSTATE depositClawState = DEPOSITCLAWSTATE.OPEN;
    public LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    //TIME
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private ElapsedTime runtime = new ElapsedTime(); // Independent timer
    private ElapsedTime liftUpTimeout = new ElapsedTime();

    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    // COLOR LIST
    List<ColorRange> colorRanges = new ArrayList<>();
    public static String detectedColor;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hue;
    public float value;
    public boolean empty;

    /**
     * constructor
     */
    public FiniteStateMachineDeposit(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intake) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intake = intake;
        runtime.reset(); // Reset timer when the arm control object is created
    }

    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        /**
         robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
         robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
         robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
         robot.liftMotorRight.setPower(0.1);
         */
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);

        /** create a list color threshold ranges*/
        colorRanges.add(new ColorRange("Black", 163, 166));
        colorRanges.add(new ColorRange("Red", 15, 25));
        colorRanges.add(new ColorRange("Blue", 220, 230));
        colorRanges.add(new ColorRange("Yellow", 75, 85));
    }

    // Deposit Arm Control
    public void DepositArmLoop() {
        long currentTime = System.currentTimeMillis();
        /** determine the Color */
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                RobotActionConfig.hsvValues);
        hue = RobotActionConfig.hsvValues[0];
        value = RobotActionConfig.hsvValues[2];

        //detect the color
        detectedColor = "None";
        empty = true;

        //LOOP THROUGH THE colorRange to check the color
        for (ColorRange range : colorRanges) {
            if (hue > range.hueMin && hue < range.hueMax) {
                detectedColor = range.colorName;
                empty = false;
                break;
            }
        }

        /** FSM Loop*/
        switch (liftState) {
            case LIFT_START:
                /** DECIDE THE COLOR FOR HIGH BASKET LIFT TRANSFER - Button X - Debounce the button press X for starting the lift extend */
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced()) {
                    liftTimer.reset();
                    liftUpTimeout.reset();
                    liftState = LIFTSTATE.LIFT_SAMPLE_BRANCH;
                }

                // "Y" button to set deposit arm to hook specimen position
                if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced()) {
                    liftTimer.reset();
                    liftUpTimeout.reset();
                    liftState = LIFTSTATE.LIFT_SPECIMEN_BRANCH;
                }

                // "Right trigger + Y" button to set deposit arm to hook specimen position
                if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6) ||
                        (gamepad_2.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6)) &&
                        isButtonDebounced()) {
                    liftTimer.reset();
                    liftState = LIFTSTATE.SPECIMEN_PICK;
                }
                break;

            case LIFT_SAMPLE_BRANCH:
                // yellow color to high basket
                if (detectedColor.equals("Yellow") || empty) {
                    liftState = LIFTSTATE.LIFT_HIGHBASKET;
                    liftTimer.reset();
                }
                // Blue and Red for choose again -- x for high basket, y for dropping
                if (detectedColor.equals("Blue") || detectedColor.equals("Red") && ((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced()) {
                    liftState = LIFTSTATE.LIFT_HIGHBASKET;
                    liftTimer.reset();
                }
                break;

            case LIFT_SPECIMEN_BRANCH:
                if (detectedColor.equals("Blue") || detectedColor.equals("Red") || detectedColor.equals("Black")) {
                    if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                            (gamepad_2.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                            isButtonDebounced()) {

                        liftState = LIFTSTATE.LIFT_HIGHBAR;
                        liftTimer.reset();
                    }
                }
                break;

            case LIFT_HIGHBASKET:
                if (!detectedColor.equals("Black")) {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_highbasketpause);
                    if (liftTimer.seconds() >= 0.25) {
                        slidesToHeightMM(RobotActionConfig.deposit_Slide_Highbasket_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
                        // Move deposit Arm & wrist servo to dump prep position
                        if (liftTimer.seconds() >= 0.5) {
                            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
                            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                            liftTimer.reset();
                            liftUpTimeout.reset();
                            liftState = LIFTSTATE.LIFT_SAMPLE_EXTEND;
                        }
                    }
                } else {
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;

            case LIFT_SAMPLE_EXTEND:
                // Check if the lift has reached the high position
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos)) {
                    //move deposit arm to dump
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_SAMPLE_DUMP;
                }
                break;

            case LIFT_SAMPLE_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime) {
                    depositClawState = DEPOSITCLAWSTATE.OPEN;
                }
                if (liftTimer.seconds() >= RobotActionConfig.postDumpTime) {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_RETRACT;
                }
                break;

            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if (Servo_AtPosition(RobotActionConfig.deposit_Claw_Open) && liftTimer.seconds() >= RobotActionConfig.retractTime) {
                    slidesToHeightMM(RobotActionConfig.deposit_Slide_Down_Pos, RobotActionConfig.deposit_Slide_DownLiftPower);
                    if (IsLiftDownAtPosition(RobotActionConfig.deposit_Slide_Down_Pos) || LSisPressed(currentTime)
                    ) {
                        robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                        robot.liftMotorRight.setPower(0);
                        liftState = LIFTSTATE.LIFT_START;
                    }
                }
                break;

            /**  2nd branch for specimen*/

            case SPECIMEN_PICK:
                if (!empty) {
                    try {
                        sleep((long) RobotActionConfig.pickTime * 1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    depositClawState = DEPOSITCLAWSTATE.CLOSE;
                }

                /** manual control can close off the claw and then move to the LIFT.HIGHBAR state*/
                if (depositClawState == DEPOSITCLAWSTATE.CLOSE) {
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_HIGHBAR;
                }
                break;
            case LIFT_HIGHBAR:
                //driveStrafe(RobotActionConfig.strafeDist);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);                                            // move the intake away to give space to specimen hook
                if (liftTimer.seconds() > RobotActionConfig.waitTime) {
                    slidesToHeightMM(RobotActionConfig.deposit_Slide_Highbar_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);        // rise up lift
                    if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbar_Pos)) {                                            // if vertical slide reached the position
                        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);                                      // set the deposit arm and deposit wrist to hook position.
                        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                        liftState = LIFTSTATE.LIFT_SPECIMEN_HOOK;
                    }
                }
                break;

            case LIFT_SPECIMEN_HOOK:
                // using manual control the claw. when specimen is hooked, manually open the claw; then deposit claw flat out.
                // After deposit claw flat out, Robot will move backward automatically.
                // Specimen hook action is achieved in two states:
                // LIFT_SPECIMEN_HOOK ---->  manual HOOK;
                if (depositClawState == DEPOSITCLAWSTATE.OPEN) { //when claw is open, means
                    liftState = LIFTSTATE.LIFT_SPECIMEN_SCORE;
                    liftTimer.reset();
                }
                break;

            case LIFT_SPECIMEN_SCORE:
                // LIFT_SPECIMEN_SCORE ----> flat out and auto back out.
                robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);                                      // set deposit wrist flat
                if (liftTimer.seconds() > RobotActionConfig.waitTime) {                                                              // wait 0.2s
                    driveBackward(RobotActionConfig.backwardDist);                                                                  // Auto drive back
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    liftState = LIFTSTATE.LIFT_RETRACT;
                    liftTimer.reset();
                }
                break;

            default:
                telemetry.addData("Error", "Unexpected state in liftState: " + liftState);
                liftState = LIFTSTATE.LIFT_START;
                break;
        }

        //Global Control ----> Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B))
                && isButtonDebounced()) {
            depositClawState = DEPOSITCLAWSTATE.OPEN;
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
            robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
            robot.liftMotorRight.setPower(0);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            liftState = LIFTSTATE.LIFT_START;
        }
        // Hung ----> Action active after 100 secs.
        //if (runtime.seconds() > 100){
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && isButtonDebounced()) {
            slidesToHeightMM(RobotActionConfig.deposit_Slide_Hang_Pos, RobotActionConfig.deposit_Slide_DownLiftPower);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && isButtonDebounced()) {
            slidesToHeightMM(Math.max(0, getSlidesCurrentPositionMM() - 10), RobotActionConfig.deposit_Slide_DownLiftPower);
            if (gamepad_1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN) && gamepad_1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        //Claw CONTROL  ---- GLOBAL CONTROL ----> BUTTON A
        ClawManualControl();
        DepositClawSwitch();
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean IsLiftAtPosition(int targetPosition) {
        int targetTicks = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold;
    }

    private boolean IsLiftDownAtPosition(int targetPosition) {
        int targetTicks = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2;
    }

    private boolean Servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    //Limit switch state
    private boolean LSisPressed(long currentTime) {
        if(currentTime - RobotActionConfig.lastPressedTime > RobotActionConfig.debounceDelay){
            RobotActionConfig.lastPressedTime = currentTime;
            return robot.limitSwitch.getState();
        } else{
            return false;
        }

    }

    //Claw CONTROL Handler ---- GLOBAL CONTROL ----> BUTTON A
    private void ClawManualControl() {
        if (((gamepad_1.getButton(GamepadKeys.Button.A) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6) ||
                (gamepad_2.getButton(GamepadKeys.Button.A) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6)) &&
                isButtonDebounced()) {
            ToggleDeposit();
        }
    }

    //Toggle Deposit Claw Helper Open - Close
    private void ToggleDeposit() {
        if (depositClawState == DEPOSITCLAWSTATE.OPEN) {
            depositClawState = DEPOSITCLAWSTATE.CLOSE;
        } else {
            depositClawState = DEPOSITCLAWSTATE.OPEN;
        }
    }

    //Deposit Claw Switch Helper
    private void DepositClawSwitch() {
        if (depositClawState != DEPOSITCLAWSTATE.OPEN) {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        } else {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }
    }

    // set deposit claw state outside class helper
    public void SetDepositClawState(DEPOSITCLAWSTATE state) {
        this.depositClawState = state;
    }

    // set deposit slides states
    public void SetDepositstate(LIFTSTATE state) {
        this.liftState = state;
    }

    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    //Auto drive method helper
    private void driveBackward(double distanceCm) {
        // Calculate target ticks based on distance
        double circumference = RobotActionConfig.WHEEL_DIAMETER_CM * Math.PI;
        double rotationsNeeded = distanceCm / circumference;
        int targetTicks = (int) (rotationsNeeded * RobotActionConfig.COUNTS_PER_MOTOR_GOBILDA_312 / RobotActionConfig.GEAR_RATIO);

        // Set target positions
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - targetTicks);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - targetTicks);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - targetTicks);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - targetTicks);

        // Set motors to run to position
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        robot.frontLeftMotor.setPower(0.5);
        robot.frontRightMotor.setPower(0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(0.5);

        // Wait until motion is complete
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()) {
        }
        // Stop motors and reset mode
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void slidesToHeightMM(double targetHeightMM, double power) {
        int targetSlideTicks = (int) (targetHeightMM * RobotActionConfig.TICKS_PER_MM_Slides);
        power = Range.clip(power, 0, 1);
        robot.liftMotorLeft.setTargetPosition(targetSlideTicks);
        robot.liftMotorRight.setTargetPosition(targetSlideTicks);

        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);
    }

    private double getSlidesCurrentPositionMM() {
        int currentPositionTicks = (robot.liftMotorRight.getCurrentPosition() + robot.liftMotorLeft.getCurrentPosition()) / 2;
        return (double) currentPositionTicks / RobotActionConfig.TICKS_PER_MM_Slides;
    }
}
