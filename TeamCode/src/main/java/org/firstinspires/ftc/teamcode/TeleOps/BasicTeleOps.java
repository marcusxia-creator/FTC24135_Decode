package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Config
@TeleOp(name = "TeleOps_Decode_gw", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    public enum BallHandlingState {
        IDLE,
        INTAKING,
        OFFTAKING
    }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private IntakeBall intakeBall;
    private OffTakeBall offTakeBall;
    private ColorDetection colorDetection;
    //private AprilTagUpdate aprilTagUpdate = new AprilTagUpdate(hardwareMap);

    //======================= States & Timers ===============================
    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    private boolean lBstartPressed = false;

    // === NEW: Ball Handling Objects and Shared List ===
    private BallHandlingState ballHandlingState = BallHandlingState.IDLE;
    private SharedBallList sharedBallList;
    private final double[] spindexerSlotAngles = {
            RobotActionConfig.spindexerSlot1,
            RobotActionConfig.spindexerSlot2,
            RobotActionConfig.spindexerSlot3
    };

    private HashMap<Integer, BallColor[]> aprilTagSequences = new HashMap<>();
    private List<LynxModule> allHubs;

    // --- Telemetry optimization ---
    private BallColor lastDisplayedColor = BallColor.UNKNOWN;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /// --- Robot and subsystems ---
        robot = new RobotHardware(hardwareMap);
        robot.init();
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        robot.initPinPoint();

        /// === NEW: Ball Handling Setup ===
        sharedBallList = new SharedBallList(spindexerSlotAngles);
        intakeBall = new IntakeBall(robot, gamepadCo2, sharedBallList.getBalls(), spindexerSlotAngles);
        offTakeBall = new OffTakeBall(robot, gamepadCo2, sharedBallList.getBalls());

        colorDetection = new ColorDetection(robot);
        colorDetection.startDetection();

        /// Bulk caching optimization
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

       ///  set default sequences for ball
        offTakeBall.setSequence(new BallColor[]{BallColor.UNKNOWN, BallColor.UNKNOWN,  BallColor.UNKNOWN});
        telemetry.addLine("No AprilTag sequence found!");
        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.update();
    }

    @Override
    public void start() {
        debounceTimer.reset();
        runTime.reset();
    }

    @Override
    public void loop() {

        // === LOAD APRIL TAG SEQUENCE AFTER 100s ===
        double t = runTime.seconds();
        if (t > 100.0)
            { //set sequence for offtake ball
                if (SharedColorSequence.aprilTagSequence != null && SharedColorSequence.aprilTagSequence.length > 0) {
                    offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
                    telemetry.addData("Loaded Sequence", Arrays.toString(SharedColorSequence.aprilTagSequence));
                }
            }

        // === CONTROL MODE TOGGLE ===
        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed && debounceTimer.milliseconds() > 250) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        // === DRIVE ===
        robotDrive.DriveLoop();

        if (controlState == ControlState.RUN) {
            robot.pinpointDriver.update();

            // === MASTER BALL HANDLING STATE MACHINE ===
            if (gamepadCo2.getButton(A)) {
                ballHandlingState = BallHandlingState.INTAKING;
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
            }

            if (gamepadCo2.getButton(X)) {
                ballHandlingState = BallHandlingState.OFFTAKING;
                offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_IDLE);
                intakeBall.stopIntake();
            }

            if (gamepadCo2.getButton(B)) {
                ballHandlingState = BallHandlingState.IDLE;
            }

            switch (ballHandlingState) {
                case INTAKING:
                    intakeBall.IntakeBallUpdate();
                    if (intakeBall.isFull()) {
                        ballHandlingState = BallHandlingState.IDLE;
                    }
                    break;

                case OFFTAKING:
                    offTakeBall.update();
                    if (offTakeBall.isSortingComplete()) {
                        ballHandlingState = BallHandlingState.IDLE;
                    }
                    break;

                case IDLE:
                    intakeBall.stopIntake();
                    offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_IDLE);
                    break;
            }
        }

        // === TEST MODE ===
        if (controlState == ControlState.TEST) {
            telemetry.addLine("----- Test Mode Active -----");
        }

        // === TELEMETRY ===
        telemetry.addLine("-------Odometry-------------------");
        telemetry.addData("Pinpoint X", "%.2f in", robot.pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y", "%.2f in", robot.pinpointDriver.getPosY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading", "%.2f°", robot.pinpointDriver.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("--------------Op Mode--------------");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("Ball Mode", ballHandlingState.name());
        telemetry.addData("Battery Voltage", getBatteryVoltage());

        telemetry.addLine("-----------Intake Info--------------");
        telemetry.addData("Intake State", intakeBall.getState());
        telemetry.addData("Is Color Detected?", intakeBall.isColorDetected());
        telemetry.addData("Current Slot", intakeBall.getCurrentSlot());
        telemetry.addData("Ball Count", intakeBall.getNumberOfBalls());
        telemetry.addData("Detected Color (enum)", intakeBall.getDetectedColor().name());
        // ---------- Shared Ball List Telemetry ----------
        telemetry.addLine("-----------Shared Ball Slots-----------");
        List<Ball> sharedBalls = sharedBallList.getBalls();
        for (Ball b : sharedBalls) {
            telemetry.addData(
                    "Slot " + b.getSlotPosition(),
                    "Ball=%s | Color=%s | Angle=%.2f",
                    b.hasBall() ? "YES" : "NO",
                    b.getColor().name(),
                    b.getSlotAngle()
            );
        }
        // Optimized color telemetry (only update if changed)
        BallColor currentColor = intakeBall.getDetectedColor();
        if (currentColor != lastDisplayedColor) {
            telemetry.addLine("Color changed → " + currentColor.name());
            lastDisplayedColor = currentColor;
        }

        telemetry.addLine("-----------Offtake Info--------------");
        telemetry.addData("Offtake State", offTakeBall.getState());
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
        intakeBall.stopIntake();
        offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_IDLE);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

    // === Helper Methods ===
    private void toggleControlState() {
        controlState = (controlState == ControlState.RUN)
                ? ControlState.TEST
                : ControlState.RUN;
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }
}
