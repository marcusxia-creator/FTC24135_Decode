package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

@Config
@TeleOp(name = "TeleOps_Decode_gw", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    // Master state machine for ball handling to prevent conflicts
    public enum BallHandlingState {
        IDLE,       // Not actively intaking or sorting
        INTAKING,   // Actively running the intake logic
        OFFTAKING     // Actively running the offtake/sorting logic
    }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private IntakeBall intakeBall;
    private OffTakeBall offTakeBall;
    private ColorDetection colorDetection = new ColorDetection(robot);
    private AprilTagUpdate aprilTagUpdate = new AprilTagUpdate(hardwareMap);

    //=======================States & Time & Lists========================================
    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();

    private boolean lBstartPressed = false;

    // === NEW: Ball Handling Objects and State ===
    private BallHandlingState ballHandlingState = BallHandlingState.IDLE;
    private List<Ball> sharedBallList;
    private double[] spindexerSlotAngles = {0.0, 0.46, 0.92}; // Example angles

    // =================April Tag and AprilTag Sequence===========================
    HashMap<Object, Object> aprilTagSequences = new HashMap<>();
    List<String> sequence;
    private int tagId;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /// Robot subsystem
        robot = new RobotHardware(hardwareMap);
        robot.init();
        /// Controller
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        /// Robot Drive
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        /// Pinpoint Initialization
        robot.initPinPoint();

        /// === NEW: Initialize ball handling subsystems ===
        // 1. Create the single, shared list for balls.
        sharedBallList = Collections.synchronizedList(new ArrayList<>());

        // 2. Instantiate subsystems, passing the *same* list to both.
        intakeBall = new IntakeBall(robot, gamepadCo2, sharedBallList, spindexerSlotAngles);

        offTakeBall = new OffTakeBall(robot, gamepadCo2, sharedBallList, spindexerSlotAngles);

        /// Get all hubs from the hardwareMap
        allHubs = hardwareMap.getAll(LynxModule.class);
        ///Set bulk caching mode to Auto
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        /// initialize AprilTag Sequences and assign Apriltag Sequences
        initializeAprilTagSequences();

        /// telemetry
        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.update();
    }
    @Override
    public void start() {
       ;    // ← reset your timer right when start() is called

    }

    @Override
    public void loop() {
        /// set April Tag and April Tag Sequence
        if (gamepadCo1.getButton(BACK)){
            tagId = aprilTagUpdate.getTagID();
            if (tagId != 0) {
                sequence = getSequenceByAprilTagId(tagId);
            }
        } else {
            sequence = new ArrayList<>();
        }
        /**
        /// Lynx for Motor encoder reading
        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                if (hub.equals(allHubs.get(0))) {
                    telemetry.addData("Drive FL Pos", bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber()));
                    telemetry.addData("Drive FR Pos", bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber()));
                    telemetry.addData("Drive BL Pos", bulkData.getMotorCurrentPosition(robot.backLeftMotor.getPortNumber()));
                    telemetry.addData("Drive BR Pos", bulkData.getMotorCurrentPosition(robot.backRightMotor.getPortNumber()));
                } else {
                    telemetry.addData("Lift L Pos", bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber()));
                    telemetry.addData("Lift R Pos", bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber()));
                }
            }
        }
        */

        ///  Start Button && LEFT BUMPER for Control State toggle
        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        /// Drive train control
        robotDrive.DriveLoop();

        /// Control condition -  Check Run Status
        if (controlState == ControlState.RUN) {
            robot.pinpointDriver.update();
            /// === NEW: Master State Machine for Ball Handling ===
            /// Use Gamepad 2 for ball controls to separate from driving
            /// Press 'A' to start intaking
            if (gamepadCo2.getButton(A)) {
                ballHandlingState = BallHandlingState.INTAKING;
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
            }

            /**Press 'X' to start sorting*/
            if (gamepadCo2.getButton(X)) {
               // offTakeBall.setRequiredSequence(Arrays.asList("Purple", "Green","Purple"));
                ballHandlingState = BallHandlingState.OFFTAKING;
                offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.READY);
                intakeBall.stopIntake(); // Ensure intake motor is off before sorting
            }

            /// to manually switch back to IDLE if needed.
            if (gamepadCo2.getButton(B)) {
                ballHandlingState = BallHandlingState.IDLE;
            }

            /// The core of the state machine. Only one case will run per loop.
            switch (ballHandlingState) {
                case INTAKING:
                    intakeBall.IntkaeBallUpdate();
                    if(intakeBall.isFull()) {
                        ballHandlingState = BallHandlingState.IDLE; // Or INTAKING, your choice
                    }
                    // NOTE: You could add a button press here (e.g., gamepadCo2.getButton(B))
                    break;

                case OFFTAKING:
                    offTakeBall.setRequiredSequence(sequence);
                    offTakeBall.update();
                    // Check if the sorting process has finished
                    if (offTakeBall.isSortingComplete()) {
                        ballHandlingState = BallHandlingState.IDLE; // Or INTAKING, your choice
                    }
                    break;

                case IDLE:
                    // Do nothing related to ball handling.
                    // Motors for intake/spindexer should be off.
                    intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_FULL);
                    intakeBall.stopIntake(); // Optional: ensure motors are off
                    offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.READY);
                    offTakeBall.stopShootBall(); // Optional: ensure motors are off
                    break;
            }
            // =================================================
        }
        /// Control condition -  Check TEST Status for Servo Test
        if (controlState == ControlState.TEST) {
            telemetry.addLine("-----No Code-----------");
            telemetry.addLine("-----No Code-----------");
            telemetry.addLine("-----No Code-----------");
        }
        /// Telemetry
        telemetry.addLine("-------Odometry-------------------");
        telemetry.addData("Pinpoint X", "%.2f inches", robot.pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y", "%.2f inches", robot.pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading", "%.2f degrees", robot.pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("-------Shooter Motor-------------------");
        telemetry.addData("Shooter Sequence", sequence);
        telemetry.addData("Shooter Power", robot.shooterMotor.getPower());
        telemetry.addLine("-------Intake Motor Motor-------------------");
        telemetry.addData("Intake Motor Power", robot.intakeMotor.getPower());
        telemetry.addData("Number of Balls", intakeBall.getNumberOfBalls());
        telemetry.addData("Empty Slot Id", intakeBall.findEmptySlot());
        telemetry.addData("color sensor depth", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("color sensor color", colorDetection.getStableColor());
        telemetry.addData("shared Ball List", sharedBallList);
        telemetry.addLine("--------------Op Mode--------------");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("Intake State", intakeBall.state());
        telemetry.addData("offTake State", offTakeBall.state());
        telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Battery Voltage", getBatteryVoltage());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.shooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }
    /// Helper -- toggle control state - RUN vs TEST
    private void toggleControlState() {
        controlState = (controlState == ControlState.RUN) ? ControlState.TEST : ControlState.RUN;
    }
    /// Helper -- to Button Debouncer
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    /// Add a voltage sensor
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        // If we never found a positive reading, default to 0
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }

    private void initializeAprilTagSequences(){
        aprilTagSequences.put(21,Arrays.asList("Green", "Purple", "Purple"));
        aprilTagSequences.put(22,Arrays.asList("Purple", "Green","Purple"));
        aprilTagSequences.put(23,Arrays.asList("Purple","Purple","Green"));
    }

    // --- NEW: Method to set sequence based on a detected AprilTag ID ---
    private List<String> getSequenceByAprilTagId(int tagId) {
        List<String> sequence;
        if (aprilTagSequences.containsKey(tagId)) {
            sequence = (List<String>) aprilTagSequences.get(tagId);
        } else {
            // Handle case where the AprilTag ID is not in our lookup table
            // For example, do nothing or set a default sequence
            sequence = Arrays.asList("Green", "Purple", "Purple");
        }
        return sequence;
    }

}
