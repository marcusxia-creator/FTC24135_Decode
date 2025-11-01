package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Vision.AprilTagUpdate;

import java.util.Arrays;
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
    public enum IsApriTagSequence{
        TRUE,
        FALSE
    }

    public enum AllianceSide {
        BLUE,
        RED
    }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private IntakeBall intakeBall;
    private OffTakeBall offTakeBall;
    private ColorDetection colorDetection;
    ///  AprilTag sequence setup
    private AprilTagUpdate aprilTagUpdate;
    private boolean useAprilTagSequence;
    private IsApriTagSequence isApriTagSequence = IsApriTagSequence.FALSE;
    private AllianceSide allianceSide;


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

    private List<LynxModule> allHubs;

    private BallColor lastDisplayedColor = BallColor.UNKNOWN;

    Pose2D targetoGoalPos = new Pose2D(DistanceUnit.INCH,72,72,AngleUnit.DEGREES,45);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /// -------- Robot and subsystems -----------
        robot = new RobotHardware(hardwareMap);
        robot.init();
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();
        ///----odometery initialization----------
        robot.initPinPoint();

        /// === NEW: Ball Handling Setup ===
        sharedBallList = new SharedBallList(spindexerSlotAngles);
        intakeBall = new IntakeBall(robot, gamepadCo2, sharedBallList.getBalls(), spindexerSlotAngles);
        offTakeBall = new OffTakeBall(robot, gamepadCo2, sharedBallList.getBalls());
        robot.spindexerServo.setPosition(spindexerSlotAngles[0]);
        robot.leftGateServo.setPosition(RobotActionConfig.GATEUP);
        robot.rightGateServo.setPosition(RobotActionConfig.GATEUP);

        ///  color sensor set up
        colorDetection = new ColorDetection(robot);
        colorDetection.startDetection();

        /// April Tag
        aprilTagUpdate = new AprilTagUpdate(hardwareMap);
        telemetry.addLine("AprilTag camera initialized.");

        /// Default Shared Color Sequence
        SharedColorSequence.aprilTagSequence = new BallColor[]{
                BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN
        };

        /// Alliance Side Default
        allianceSide = AllianceSide.RED;

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
    public void init_loop() {
        aprilTagUpdate.update();
        int detectedID = aprilTagUpdate.getTagID();
        BallColor[] tagSequence = aprilTagUpdate.getSequence();
        if (detectedID != -1 && tagSequence != null) {
            SharedColorSequence.aprilTagSequence = tagSequence;
            telemetry.addData("AprilTag ID", detectedID);
            telemetry.addData("Detected Sequence", aprilTagUpdate.getSequenceAsString());
        } else {
            telemetry.addLine("No valid AprilTag detected yet.");
        }

        // Toggle the Alliance Side
        if (gamepadCo1.getButton(BACK)&&gamepadCo1.getButton(LEFT_BUMPER)&&!lBstartPressed&& debounceTimer.milliseconds()>DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            lBstartPressed = true;
            toggleAllianceSide();
            if (allianceSide == AllianceSide.BLUE) {
                targetoGoalPos = new Pose2D( DistanceUnit.INCH,-70,-70, AngleUnit.DEGREES,45);
            }else{
                targetoGoalPos = new Pose2D( DistanceUnit.INCH,-70,70, AngleUnit.DEGREES,-45);
            }
        }
        else if (!gamepadCo1.getButton(BACK) || !gamepadCo1.getButton(LEFT_BUMPER)) {
                lBstartPressed = false;
            }
        telemetry.addData("Alliance Side", allianceSide.name());
    }

    @Override
    public void start() {
        debounceTimer.reset();
        runTime.reset();
    }

    @Override
    public void loop() {

        // === Decide if AprilTag sequence will be used ===
        if (gamepadCo2.getButton(GamepadKeys.Button.DPAD_UP)&&debounceTimer.milliseconds()>DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            toggleAprilTagSequenceState();
            if (isApriTagSequence == IsApriTagSequence.TRUE) {
                offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
            }else{
                offTakeBall.setSequence(null);
            }
        }
        // === LOAD APRIL TAG SEQUENCE AFTER 100s ===
        double t = runTime.seconds();
        if (t > 100.0 && !useAprilTagSequence)
            { //set sequence for offtake ball
                useAprilTagSequence = true;
                offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
            }

        /// === CONTROL MODE TOGGLE ===
        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed && debounceTimer.milliseconds() > DEBOUNCE_THRESHOLD) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        /// === DRIVE ===
        robotDrive.DriveLoop();

        /// === RUn mode ===
        if (controlState == ControlState.RUN) {
            // === UPDATE ODOMETRY ===
            robot.pinpointDriver.update();
            // === CALCULATE TARGET DISTANCE ===
            offTakeBall.setDistanceToGoal(getTargetGoalDist(targetoGoalPos));

            // === MASTER BALL HANDLING STATE MACHINE ===
            if (gamepadCo2.getButton(A)&& debounceTimer.milliseconds()>DEBOUNCE_THRESHOLD) {
                debounceTimer.reset();
                ballHandlingState = BallHandlingState.INTAKING;
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
            }

            if (gamepadCo2.getButton(X) && debounceTimer.milliseconds()>DEBOUNCE_THRESHOLD) {
                debounceTimer.reset();
                ballHandlingState = BallHandlingState.OFFTAKING;
                offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_IDLE);
                intakeBall.stopIntake();
            }

            if (gamepadCo2.getButton(B)&& debounceTimer.milliseconds()>DEBOUNCE_THRESHOLD) {
                debounceTimer.reset();
                ballHandlingState = BallHandlingState.IDLE;
            }

            /// ===== FSM MAIN CONTROL STATE =====
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
        /** =================================== TELEMETRY =================================================== */
        telemetry.addLine("-------Odometry-------------------");
        telemetry.addData("Pinpoint X", "%.2f in", robot.pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y", "%.2f in", robot.pinpointDriver.getPosY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading", "%.2f°", robot.pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distance", getTargetGoalDist(targetoGoalPos));
        telemetry.addLine("--------------Op Mode--------------");
        telemetry.addData("Run Time", "%.1f sec",t);
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
        telemetry.addData("Detected Distance", colorDetection.getDistance());
        /// ---------- Shared Ball List Telemetry ----------------------------------------------------------
        telemetry.addLine("-----------Shared Ball Slots-----------");
        List<Ball> sharedBalls = sharedBallList.getBalls();
        for (Ball b : sharedBalls) {
            telemetry.addData(
                    "Slot " + b.getSlotPosition(),
                    "Ball=%s | Color=%s | Angle=%.2f",
                    b.hasBall() ? "YES" : "NO",
                    b.getColor().name(),
                    b.getSlotAngle());}
        /// ---------- Optimized color telemetry (only update if changed) ------------------------------------
        BallColor currentColor = intakeBall.getDetectedColor();
        if (currentColor != lastDisplayedColor) {
            telemetry.addLine("Color changed → " + currentColor.name());
            lastDisplayedColor = currentColor;
        }
        /// ===================================================================================================
        telemetry.addLine("-----------Offtake Info--------------");
        telemetry.addData("Offtake State", offTakeBall.getState());
        telemetry.addData("Offtake color used", offTakeBall.isColorSequence());
        telemetry.addData("Offtake Shooter power", offTakeBall.getShooterPower());
        /// ===================================================================================================
        telemetry.addLine("-----------AprilTag Info--------------");
        telemetry.addData("Tag ID", aprilTagUpdate.getTagID());
        telemetry.addData("Sequence Mode", useAprilTagSequence ? "TAG-GUIDED" : "FREE-SHOOTING");
        telemetry.addData("Sequence Loaded", Arrays.toString(SharedColorSequence.aprilTagSequence));
        telemetry.addData("Field Coord", aprilTagUpdate.getRobotFieldCoordinateAsString());
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
    private void toggleAprilTagSequenceState() {
        isApriTagSequence = (isApriTagSequence == IsApriTagSequence.TRUE)
                ? IsApriTagSequence.FALSE
                : IsApriTagSequence.TRUE;
    }
    private void toggleAllianceSide() {
        allianceSide = (allianceSide == AllianceSide.BLUE)
                ? AllianceSide.BLUE
                : AllianceSide.RED;
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
    public double getTargetGoalDist(Pose2D targetGoalPos){
        double tag_x = targetGoalPos.getX(DistanceUnit.INCH);
        double tag_y = targetGoalPos.getY(DistanceUnit.INCH);
        double rob_x = robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        double rob_y = robot.pinpointDriver.getPosY(DistanceUnit.INCH);
        return Math.sqrt(Math.pow(Math.abs(tag_x - rob_x),2) + Math.pow(Math.abs(tag_y - rob_y),2));
    }
}
