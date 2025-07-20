package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/** config
 * deposit Left Arm Position initial position for installation = 0
 * deposit Left Arm Position initial position = 0
 *
 */

/** Control Config
 * TeleOps Control - Global Control Button
 *  * LEFT BUMPER + START           ----> control state -  Run vs ServoTest -- GAMEPAD #1 ONLY
 *  * LEFT STICK BUTTON             ----> control state -  SEMI-AUTO TRIGGER -- THEN INTO AUTO DRIVE FOR SPECIMEN SCORING -- GAMEPAD #1 ONLY
 *  * RIGHT STICK BUTTON            ----> control state -  SEMI-AUTO CANCEL  -- CANCEL THE AUTO DRIVE FOR SPECIMEN SCORING -- GAMEPAD #1 ONLY
 *  * BACK BUTTON ALONE             ----> reset the vertical slide AND DEPOSIT ARM BACK TO TRANSFER at the start of the TeleOps
 * -----------------------------------------------------------------------------------------------
 * DRIVETRAIN Control - Global Control Button
 *  * Right STICK                   ---->  Y and X movement and diagonal
 *  * Left STICK                    ---->  Turn
 *  * START                         ---->  Field and Robot centric selection
 *  * BACK                          ---->  reset IMU Yaw Angle
 *  * LEFT Trigger + DRVING JOYSTICKs VALUE ---->  fine movement - depending on the pressed level (0.2 - 0.8)
 * -----------------------------------------------------------------------------------------------
 * DEPOSIT ARM
 * --- LOCAL STATE
 *  * X                             ---->high basket drop series - local state - LIFT_START
 *  * Y                             ---->deposit arm flip to the back side - local state - LIFT_START
 * --- GLOBAL STATE
 *  * B                             ----> TO CANCEL DEPOSIT SYSTEM
 *  *                               ----> SLIDE AND DEPOSIT WRIST AND DEPOSIT ARM BACK TO "TRANSFER POSITION"
 *  * Right Trigger + A             ----> TO TOGGLE DEPOSIT CLAW OPEN.CLOSE
 *-----------------------------------------------------------------------------------------------
 *  INTAKE ARM
 *  * --- LOCAL STATE - INTAKE PICK
 *  * DPAD_RIGHT                    ----> intake extend and set pick position action series - local state - INTAKE_START
 *  * DPAD_LEFT                     ----> intake retract - local state - INTAKE_PICK
 *  * LEFT_BUMPER / RIGHT_BUMPER    ---->  for INTAKE CLAW ROTATION
 *  * DPAD_UP / DPAD_DOWN           ---->  for INTAKE ARM UP AND DOWN
 *  -----------------------------------------------------------------------------------------------
 *  * --- GLOBAL STATE
 *  * Right Trigger + DPAD_RIGHT    ----> to lower the intake arm for near side pick up
 *  * A                             ----> TO TOGGLE INTAKE CLAW OPEN/CLOSE
 *  * Right bumper + A              ----> To TOGGLE DEPOSIT CLAW OPEN/CLOSE
*/
@Disabled
@TeleOp(name = "TeleOps_Champion_Prep_SemiAuto", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps_SemiAuto extends OpMode {

    //Control State Variable
    public enum ControlState {
        DRIVE_CONTROL,
        TEST,
        AUTOMATIC_CONTROL
    }

    //Robot
    public RobotHardware robot;                             // Bring in robot hardware configuration
    public GamepadEx gamepadCo1;                            //For gamepad
    public GamepadEx gamepadCo2;

    //Robot drive
    public RobotDrive robotDrive;                           //For robot drive
    public SampleMecanumDriveCancelable drive;                        //For robot semiAuto drive

    //Robot Intake & Deposit
    public FiniteStateMachineDeposit depositArmDrive;       //For Robot Deposit Arm
    public FiniteStateMachineIntake intakeArmDrive;         //For Robot Intake

    public ServoTest servoTest;                             //For Servo Testing

    private ControlState controlState = ControlState.DRIVE_CONTROL;   // Control State

    private ElapsedTime debounceTimer = new ElapsedTime();  // Timer for debouncing

    private boolean lBstartPressed = false;                 // For Button State
    private boolean autoPressed = false;                    // for auto control combination buttons states

    //Bulk Reading
    private List<LynxModule> allHubs;                       // Bulk Reading

    //for color
    private String detectedColor;

    public static boolean initialRun;

    private AutoDriveHandlerOldSpecimen autoDriveHandler;

    //For time
    double oldTime = 0;

    @Override
    public void init() {

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Initialize RR drive
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        // Initialize hardware in RobotHardware
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        //gamepad
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //robotDrive
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);   // Pass robot instance to RobotDrive
        robotDrive.Init();                                                              // Initialize RobotDrive

        //Deposit Arm control
        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry); // Pass parameters as needed);

        //Intake Arm Control
        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1,gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        //Servo Testing
        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);
        //servoTest.ServoTestInit();

        // get bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //Robot Drive State
        RobotDrive.DriveMode currentDriveMode = robotDrive.getDriveMode();

        //Reset the motor encoder

        /** transfer the currentPose from end of Auto -- each Auto code need to
         * add PoseStorage.currentPose = drive.getPoseEstimate(); at the end of the AutoCode
         *
         * */
        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));// this is for manual testing.
        drive.setPoseEstimate(startPose);
        //drive.setPoseEstimate(PoseStorage.currentPose);
        initialRun = true; //For specimen semi-auto control

        //// Initialized an AutoHandler
        autoDriveHandler = new AutoDriveHandlerOldSpecimen(drive,robot, 1,depositArmDrive);

        //Telemetry
        telemetry.addLine("-------------------");
        telemetry.addData("Status", " initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addData("Control Mode", controlState.name());
        telemetry.addData("Drive Mode", currentDriveMode.name());
        telemetry.addLine("-------------------");
        telemetry.addData("Vertical slide Encoder_left",robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Vertical slide Encoder_right",robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
        resetRuntime();
        }

    @Override
    public void loop () {
        //update the pose through the drive -roadrunner.
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d pinpointPose = drive.updatePinpointPosition();
        // Update pose dynamically in AutoDriveHandler
        autoDriveHandler.updatePoseEstimate(poseEstimate);

        //reset pose
        if (gamepadCo2.getButton(BACK) && debounceTimer.seconds() > 0.2) {
            debounceTimer.reset();
            Pose2d startPose = new Pose2d(0, -32, Math.toRadians(-90));// this is for manual testing.
            drive.setPoseEstimate(startPose);
            initialRun = true;
        }


        // Button B to reset vertical slide position to bottom.
        if (gamepadCo1.getButton(BACK) && debounceTimer.seconds()>0.2){
            debounceTimer.reset();
            robot.liftMotorLeft.setTargetPosition(0);
            robot.liftMotorRight.setTargetPosition(0);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(0.3);                                          // Make sure lift motor is on
            robot.liftMotorRight.setPower(0.3);
            while (robot.liftMotorLeft.isBusy()&&robot.liftMotorRight.isBusy()){
                    robot.liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    break;
            }
            // when position is back to 0, deposit is initialized.
            depositArmDrive.Init();
        }

        //Bulk Reading for Motors
        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                // Example: Reading motor position for each hub
                if (hub.equals(allHubs.get(0))) { // Assuming the first hub is Control Hub
                    int frontLeftMotor = bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber());
                    int frontRightMotor = bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber());
                    int backLeftMotor = bulkData.getMotorCurrentPosition(robot.backLeftMotor.getPortNumber());
                    int backRightMotor = bulkData.getMotorCurrentPosition(robot.backRightMotor.getPortNumber());
                    telemetry.addData("Drive Motor FL Motor (Control Hub) Position", frontLeftMotor);
                    telemetry.addData("Drive Motor FR Motor (Control Hub) Position", frontRightMotor);
                    telemetry.addData("Drive Motor BL Motor (Control Hub) Position", backLeftMotor);
                    telemetry.addData("Drive Motor BR Motor (Control Hub) Position", backRightMotor);
                } else if (hub.equals(allHubs.get(1))) { // Assuming the second hub is Expansion Hub
                    int liftLeftMotor = bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber());
                    int  liftRightMotor= bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber());
                    telemetry.addData("Deposit Left Motor Position (Expansion Hub)", liftLeftMotor);
                    telemetry.addData("Deposit right Motor Position (Expansion Hub)", liftRightMotor);
                }
            }
        }

        // Robot Drivetrain
        RobotDrive.DriveMode currentDriveMode = robotDrive.getDriveMode();

        //Control Mode Selection
        if ((gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER)) && !lBstartPressed) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!(gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER))) {
            lBstartPressed = false;
        }

        //RUN Mode Selection
        switch (controlState) {
            case DRIVE_CONTROL:
                robotDrive.DriveLoop(); // Use RobotDrive methods to drive the robot
                //Deposit Arm Control
                depositArmDrive.DepositArmLoop();
                FiniteStateMachineDeposit.LIFTSTATE liftState = depositArmDrive.liftState;
                FiniteStateMachineDeposit.DEPOSITCLAWSTATE depositClawState = depositArmDrive.depositClawState;
                //Intake Arm Control
                intakeArmDrive.IntakeArmLoop();
                FiniteStateMachineIntake.INTAKESTATE intakeState = intakeArmDrive.intakeState;
                FiniteStateMachineIntake.INTAKECLAWSTATE intakeClawState = intakeArmDrive.intakeClawState;
                telemetry.addLine("---------------------");
                telemetry.addData("Deposit State", liftState);
                telemetry.addData("Deposit Claw State", depositClawState);
                telemetry.addLine("---------------------");
                telemetry.addData("Intake State", intakeState);
                telemetry.addData("Intake Claw State", intakeClawState);
                telemetry.addLine("---------------------");
                telemetry.addData("Detected Color", detectedColor);
                //telemetry.addData("Color Sensor value", RobotActionConfig.hsvValues[2]);

                /** AutoMode Control*/
                if ((gamepadCo1.getButton(LEFT_STICK_BUTTON) && !autoPressed && isButtonDebounced())){
                    /**Global Control ----> Handle Auto Drive if 'LeftSTICK' button is pressed*/
                    autoPressed = true;
                    if(autoDriveHandler.handleButtonY()){
                        initialRun = false;
                        controlState = ControlState.AUTOMATIC_CONTROL;
                    }
                } else if (!gamepadCo1.getButton(LEFT_STICK_BUTTON)) {
                        autoPressed = false;
                }
                break;

            case AUTOMATIC_CONTROL:
                //State Control ----> Handle Auto Cancel Action if 'Right_Stick Button' button is pressed
                if (gamepadCo1.getButton(RIGHT_STICK_BUTTON) && isButtonDebounced()) {
                    drive.breakFollowing();
                    initialRun = true;
                    controlState = ControlState.DRIVE_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    controlState = ControlState.DRIVE_CONTROL;
                }
                break;
        }

        //Refresh frequency
        long currentTime = System.currentTimeMillis();
        //double newTime = getRuntime();
        double newTime = currentTime;
        double loopTime = newTime-oldTime;
        double frequency = 1000/loopTime;
        oldTime = newTime;

        // Telemetry
        telemetry.addData("Run Mode", controlState.name());
        telemetry.addData("Drive state",drive.isBusy());
        telemetry.addLine("---------------------");
        telemetry.addData("Drive Mode", currentDriveMode.name());

        telemetry.addLine("---------------------");
        telemetry.addData("Deposit Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());

        telemetry.addLine("---------------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide LEFT Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide RIGHT Position", robot.intakeRightSlideServo.getPosition());

        telemetry.addLine("---------------------");
        telemetry.addData("Heading ", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addLine("---------------------");
        telemetry.addData("Auto Initial Run",initialRun);
        telemetry.addData("PoseEstimate",poseEstimate);
        telemetry.addData("Pinpoint Pose",pinpointPose);
        /**
        telemetry.addLine("---------Frequency--------");
        telemetry.addData("Pinpoint Frequency", drive.pinPointFrequency()); //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
         /
        telemetry.update();*/
    }

    //Stop the Robot
    public void stop () {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

    //Control State Toggle
    private void toggleControlState () {
        if (controlState != ControlState.DRIVE_CONTROL) {
            controlState = ControlState.DRIVE_CONTROL;
        } else {
            controlState = ControlState.TEST;
        }
    }

    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}
