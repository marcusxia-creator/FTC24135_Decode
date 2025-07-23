package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;


/** Button Config for deposit
 * X                           : high basket extend State           - LOCAL STATE - LIFT_START
 * * ->X                         : dump                             - LOCAL STATE - LIFT_SAMPLE_DUMP
 * Y                           : Specimen score                     - LOCAL STATE - LIFT_START
 *
 * B                           : Cancel;back to transfer pos        - GLOBAL STATE
 * A                           : TOGGLE DEPOSIT CLAW OPEN/CLOSE     - GLOBAL STATE
 * *DPAD UP && LEFT BUMPER      : Hung                              - GLOBAL STATE -ACTIVE AFTER 100S
 * *DPAD DOWN && LEFT BUMPER    : Hung DOWN - 300 TICK EVERY PRESS  - GLOBAL STATE -ACTIVE AFTER 100S
 * *DPAD UP && Right Trigger      :Specimen Score, raise up the vertical slide to hook the specimen.
 */

public class FiniteStateMachineDepositPID {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    ///bring in the finitemachinestateintake
    private final FiniteStateMachineIntake intake;
    private Telemetry telemetry;
    /**
     * add PID slider pid helper & parameters
     */
    private SlidesPIDControl slidePIDControl;
    private static final double KP = 0.1, KI = 0.001, KD = 0.01;
    private static final double FULL_RANGE_TICKS = RobotActionConfig.TICKS_PER_MM_SLIDES*RobotActionConfig.deposit_Slide_Highbasket_Pos; // or use RobotActionConfig

    /**
     * Deposit Arm State
     */
    public enum LIFTSTATE {
        LIFT_START,
        LIFT_SAMPLE_BRANCH,
        LIFT_HIGHBASKET,
        LIFT_SAMPLE_EXTEND,
        LIFT_SAMPLE_DUMP,
        LIFT_RETRACT_PAUSE,
        LIFT_RETRACT,
        LIFT_WALL_PICK,
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

    /**
     * member DECLEAR
     */
    // STATE
    public DEPOSITCLAWSTATE depositClawState = DEPOSITCLAWSTATE.OPEN;
    public LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    //TIME
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private ElapsedTime holdTimer = new ElapsedTime(); // Timer for limit switch debouncing
    private ElapsedTime runtime = new ElapsedTime(); // Independent timer
    private ElapsedTime hangtime = new ElapsedTime();
    private ElapsedTime liftUpTimeout = new ElapsedTime();
    private ElapsedTime VSRetractTimer = new ElapsedTime();


    // COLOR LIST
    List<ColorRange> colorRanges = new ArrayList<>();
    //public String detectedColor = "None";

    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hue;
    public float value;
    public boolean empty =true;
    static float hsvValues[] = {0F,0F,0F};                                   // set color sensor value

    /**
     * constructor
     */
    public FiniteStateMachineDepositPID(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intake, Telemetry telemetry) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intake = intake;
        runtime.reset(); // Reset timer when the arm control object is created
        slidePIDControl = new SlidesPIDControl(robot,KP,KI,KD,FULL_RANGE_TICKS);
    }

    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        depositClawState = DEPOSITCLAWSTATE.OPEN;
    }

    /** create a list color threshold ranges*/

    // Deposit Arm Control
    public void DepositArmLoop() {
        ///slide PID Control Update
        slidePIDControl.update();

        /** FSM Loop*/
        switch (liftState) {
            case LIFT_START:
                /** DECIDE THE COLOR FOR HIGH BASKET LIFT TRANSFER - Button X - Debounce the button press X for starting the lift extend */
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_1.getButton(LEFT_BUMPER)) ||
                        (gamepad_2.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_2.getButton(LEFT_BUMPER))) &&
                        isButtonDebounced()) {
                    liftTimer.reset();
                    liftUpTimeout.reset();
                    liftState = LIFTSTATE.LIFT_HIGHBASKET;
                }

                // "Y" button to set deposit arm to hook specimen position
                if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_1.getButton(LEFT_STICK_BUTTON))  ||
                        (gamepad_2.getButton(GamepadKeys.Button.Y) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_2.getButton(LEFT_STICK_BUTTON))) &&
                        isButtonDebounced()) {
                    liftState = LIFTSTATE.LIFT_WALL_PICK;
                    liftTimer.reset();
                }

                break;

            case LIFT_HIGHBASKET:
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_highbasketpause);
                    if (liftTimer.seconds() >= 0.05) {
                        //slidesToHeightMM(RobotActionConfig.deposit_Slide_Highbasket_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
                        slidePIDControl.setTargetMM(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                        // Move deposit Arm & wrist servo to dump prep position
                        if (liftTimer.seconds() >= 0.5) {
                            robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
                            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                            liftTimer.reset();
                            liftUpTimeout.reset();
                            liftState = LIFTSTATE.LIFT_SAMPLE_EXTEND;
                        }
                    }
                break;
            case LIFT_SAMPLE_EXTEND:
                // Check if the lift has reached the high position
                if (slidePIDControl.atTarget()) {
                    //move deposit arm to dump
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_SAMPLE_DUMP;
                }
                break;
            case LIFT_SAMPLE_DUMP:
                // Wait for the dump time to pass
                if ((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_1.getButton(LEFT_BUMPER)) ||
                        (gamepad_2.getButton(GamepadKeys.Button.X) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_2.getButton(LEFT_BUMPER)) &&
                                isButtonDebounced()) {
                    depositClawState = DEPOSITCLAWSTATE.OPEN;
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_RETRACT_PAUSE;
                    }
                break;
            case LIFT_RETRACT_PAUSE:
                /**
                if((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_1.getButton(LEFT_BUMPER)) ||
                        (gamepad_2.getButton(GamepadKeys.Button.X) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_2.getButton(LEFT_BUMPER)) &&
                                isButtonDebounced()) {
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);}
                */
                if (liftTimer.seconds()>0.25){
                    move(RobotActionConfig.Move_Distance);// use while-loop to check motor is busy and hole the code.
                    liftState = LIFTSTATE.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if (Servo_AtPosition(RobotActionConfig.deposit_Claw_Open) && liftTimer.seconds() > 0.1) {
                    slidePIDControl.setTargetMM(RobotActionConfig.deposit_Slide_Down_Pos);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);  /// Reset servo to idle
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer); /// Reset servo to idleS
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    if (slidePIDControl.atTarget()) {
                        robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                        robot.liftMotorRight.setPower(0);
                        robot.liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        robot.liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        liftState = LIFTSTATE.LIFT_START;
                    }
                }
                break;

            /**  2nd branch for specimen*/
            case LIFT_WALL_PICK:
                /// LIFT_WALL_PICK ---->  run deposit arm to wall pick position
                slidePIDControl.setTargetMM(RobotActionConfig.deposit_Slide_Pick_Rear_Pos);
                if (liftTimer.seconds() > RobotActionConfig.waitTime) {
                        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
                        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
                        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
                    if (liftTimer.seconds() > RobotActionConfig.waitTime+0.6) {
                        depositClawState = DEPOSITCLAWSTATE.OPEN;
                        liftState = LIFTSTATE.LIFT_HIGHBAR;
                        liftTimer.reset();
                    }
                }
                break;
            case LIFT_HIGHBAR:
                /// LIFT_WALL_PICK ---->  manual control claw - Y button to close and run next state
                if (depositClawState == DEPOSITCLAWSTATE.CLOSE){
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);     /// set the deposit arm and deposit wrist to hook position.
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                    slidePIDControl.setTargetMM(RobotActionConfig.deposit_Slide_Highbar_Pos);        /// rise up lift
                }
                if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_1.getButton(LEFT_STICK_BUTTON))  ||
                        (gamepad_2.getButton(GamepadKeys.Button.Y) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && !gamepad_2.getButton(LEFT_STICK_BUTTON))) &&
                        isButtonDebounced()) {                                                      /// if vertical slide reached the position
                    liftState = LIFTSTATE.LIFT_SPECIMEN_HOOK;
                    liftTimer.reset();
                }
                break;

            case LIFT_SPECIMEN_HOOK:
                /// LIFT_SPECIMEN_HOOK ---->  score action
                    slidePIDControl.setTargetMM(RobotActionConfig.deposit_Slide_Highbar_Score_Pos);
                    if (slidePIDControl.atTarget()) {
                        depositClawState = DEPOSITCLAWSTATE.OPEN;
                        liftState = LIFTSTATE.LIFT_SPECIMEN_SCORE;
                        liftTimer.reset();}
                break;

            case LIFT_SPECIMEN_SCORE:
                ///
                if(liftTimer.seconds() > 0.5){
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    liftState = LIFTSTATE.LIFT_RETRACT;
                    liftTimer.reset();
                }
                //}
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
            robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
            robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
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
            robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_hang_Pos);
            robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_hang_Pos);
            if (hangtime.seconds() > 0.5) {
            }
        }
        if (gamepad_1.getButton(GamepadKeys.Button.BACK)
                && isButtonDebounced()) {
            slidesToHeightMM(-10000, 0.2);
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
            robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
            robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
            if (hangtime.seconds() > 2) {
                robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        //Claw CONTROL  ---- GLOBAL CONTROL ----> BUTTON A
        ClawManualControl();
        DepositClawSwitch();
    }

    // Color return helper

    // Helper method to check if the lift is within the desired position threshold
    private boolean IsLiftAtPosition(int targetPosition) {
        int targetTicks = (int) (targetPosition*RobotActionConfig.TICKS_PER_MM_SLIDES);
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold;
    }

    private boolean IsLiftDownAtPosition(int targetPosition) {
        int targetTicks = (int) (targetPosition*RobotActionConfig.TICKS_PER_MM_SLIDES);
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2;
    }

    private boolean Servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    /*
    //Limit switch state
    private boolean LSisPressed() {
        boolean switchState = robot.limitSwitch.getState(); // Read switch state

        if (switchState) {
            if (holdTimer.milliseconds() >= RobotActionConfig.debounceDelay) {
                return true; // Return true only if held for at least 200ms
            }
        } else {
            holdTimer.reset(); // Reset timer when switch is released
        }
        return false;

     */

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
    private void move(double distanceCm) {
        // Calculate target ticks based on distance
        double circumference = RobotActionConfig.WHEEL_DIAMETER_CM * Math.PI;
        double rotationsNeeded = distanceCm / circumference;
        int targetTicks = (int) (rotationsNeeded * RobotActionConfig.COUNTS_PER_MOTOR_GOBILDA_312 / RobotActionConfig.GEAR_RATIO);

        // Set target positions
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + targetTicks);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + targetTicks);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + targetTicks);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + targetTicks);

        // Set motors to run to position
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        robot.frontLeftMotor.setPower(1.0);
        robot.frontRightMotor.setPower(1.0);
        robot.backLeftMotor.setPower(1.0);
        robot.backRightMotor.setPower(1.0);

        // Wait until motion is complete
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()){
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
        int targetSlideTicks = (int) ((targetHeightMM * RobotActionConfig.TICKS_PER_MM_SLIDES));
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
        return (double) currentPositionTicks/RobotActionConfig.TICKS_PER_MM_SLIDES;
    }
}
