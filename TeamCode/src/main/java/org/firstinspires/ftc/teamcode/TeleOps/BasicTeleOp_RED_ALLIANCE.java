package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;

@TeleOp(name = "RED_TELEOP_MEET_2", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_RED_ALLIANCE extends OpMode {

    public enum RobotActionState {
        Sequence_Shooting,
        Sort_Shooting,
        Intaking,
        Idle
    }

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private GamepadInput gamepadInput;
    private RobotDrive robotDrive;

    private FSMShooter fsmShooter;
    private FSMIntake fsmIntake;

    private Spindexer spindexer;
    private SpindexerManualControl spindexerManualControl; // make sure you init or guard

    private LUTPowerCalculator shooterPowerAngleCalculator;

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private ColorDetection colorDetection;

    public RobotActionState actionStates;
    private RobotActionState lastActionState = null;

    public static Alliance alliance;

    public Limelight limelight;
    private Turret turret; // if not used, keep null and guard

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();
        robot.initExternalIMU();

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        gamepadInput = new GamepadInput(gamepadCo1, gamepadCo2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        spindexer = new Spindexer(robot,
                Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0);

        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);

        fsmShooter = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, shooterPowerAngleCalculator, gamepadInput);
        fsmShooter.Init();

        fsmIntake = new FSMIntake(gamepadCo1, gamepadCo2, robot, spindexer);

        colorDetection = new ColorDetection(robot);

        alliance = Alliance.RED_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(true);

        actionStates = RobotActionState.Idle;
        lastActionState = null; // so enter handler runs on first loop

        // turret = new Turret(robot); // if you want it
        limelight = new Limelight(robot, turret);
        limelight.initLimelight(24);
        limelight.start();

        debounceTimer.reset();
    }

    @Override
    public void loop() {
        // ----- inputs / updates -----
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();
        gamepadInput.update();

        robot.pinpoint.update();

        // If you want hue telemetry, OK:
        BallColor ballColor = BallColor.fromHue(colorDetection.getHue());

        // Guard: this will crash if not initialized
        if (spindexerManualControl != null) {
            spindexerManualControl.loop();
        }

        robotDrive.DriveLoop();

        // ----- state changes based on buttons -----
        buttonUpdate();

        // ----- on-enter handler (stop the other subsystem immediately) -----
        if (actionStates != lastActionState) {
            onActionStateEnter(actionStates);
            lastActionState = actionStates;
        }

        // ----- run EXACTLY ONE FSM -----
        switch (actionStates) {
            case Sequence_Shooting:
                fsmShooter.SequenceShooterLoop();
                break;

            case Sort_Shooting:
                fsmShooter.SortShooterLoop();
                break;

            case Intaking:
                fsmIntake.loop();
                break;

            case Idle:
            default:
                // Run nothing. Keep robot safe.
                break;
        }

        // LED alarm light
        if (shooterPowerAngleCalculator.getZone() == 0) {
            robot.LED.setPosition(0.28);
        } else {
            robot.LED.setPosition(1.0);
        }

        telemetryManager();
    }

    // ==========================
    //  Critical: stop functions
    // ==========================
    private void stopIntakeNow() {
        // Force everything intake-related OFF immediately
        if (robot != null && robot.intakeMotor != null) {
            robot.intakeMotor.setPower(0);
        }
        if (fsmIntake != null) {
            fsmIntake.intakeStates = IntakeStates.INTAKE_IDLE;
        }
    }

    private void stopShooterNow() {
        // Force everything shooter-related OFF immediately
        if (robot != null) {
            if (robot.topShooterMotor != null) robot.topShooterMotor.setPower(0);
            if (robot.bottomShooterMotor != null) robot.bottomShooterMotor.setPower(0); // if exists
            // If you have feeder/indexer motors/servos, stop them here too
        }
        if (fsmShooter != null) {
            fsmShooter.shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        }
    }

    // Called once when action state changes
    private void onActionStateEnter(RobotActionState newState) {
        switch (newState) {
            case Sequence_Shooting:
                stopIntakeNow();
                break;
            case Sort_Shooting:
                stopIntakeNow();           // ✅ ensure intake is off even though intake FSM won't run
                // Optional: reset shooter timers here if needed
                break;

            case Intaking:
                stopShooterNow();          // ✅ ensure shooter is off even though shooter FSM won't run
                // Optional: reset intake timers here if needed
                break;

            case Idle:
            default:
                stopIntakeNow();
                stopShooterNow();
                break;
        }
    }

    // ==========================
    // Debounce
    // ==========================
    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    // ==========================
    // Buttons -> actionStates
    // ==========================
    private void buttonUpdate() {

        // X -> sequence shooting
        if ((gamepadCo1.getButton(GamepadKeys.Button.X) || gamepadCo2.getButton(GamepadKeys.Button.X))
                && isButtonDebounced()) {
            actionStates = RobotActionState.Sequence_Shooting;
            fsmShooter.shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING; // or whatever your entry state is
        }

        // Y -> sort shooting
        if ((gamepadCo1.getButton(GamepadKeys.Button.Y) || gamepadCo2.getButton(GamepadKeys.Button.Y))
                && isButtonDebounced()) {
            //actionStates = RobotActionState.Sort_Shooting;
            // set any shooter entry state needed
        }

        // DPAD_LEFT -> intake
        if ((gamepadCo1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_LEFT))
                && isButtonDebounced()) {
            actionStates = RobotActionState.Intaking;
            fsmIntake.intakeStates = IntakeStates.INTAKE_PREP; // entry state
        }

        // DPAD_RIGHT -> reverse intake (stays in intake mode)
        if ((gamepadCo1.getButton(GamepadKeys.Button.DPAD_RIGHT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                && isButtonDebounced()) {
            actionStates = RobotActionState.Intaking;
            fsmIntake.intakeTimer.reset();
            fsmIntake.reversing();
        }

        // B -> idle
        if ((gamepadCo1.getButton(GamepadKeys.Button.B) || gamepadCo2.getButton(GamepadKeys.Button.B))
                && isButtonDebounced()) {
            actionStates = RobotActionState.Idle;
        }

        // DPAD_DOWN -> reset pose based on alliance (no debounce is OK)
        if (gamepadCo1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            if (alliance == Alliance.RED_ALLIANCE) {
                robot.pinpoint.setPosition(redAllianceResetPose);
                robot.LED.setPosition(0.28);
            } else {
                robot.pinpoint.setPosition(blueAllianceResetPose);
                robot.LED.setPosition(0.611);
            }
        }
    }

    // ==========================
    // Telemetry (guard nulls)
    // ==========================
    public void telemetryManager() {
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Intake State", (fsmIntake != null) ? fsmIntake.intakeStates : "null");
        telemetry.addData("Shooter State", (fsmShooter != null) ? fsmShooter.shooterState : "null");

        if (robot != null && robot.distanceSensor != null) {
            telemetry.addData("Sensor Distance", robot.distanceSensor.getDistance(DistanceUnit.MM));
        }

        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("Heading", robot.pinpoint.getHeading(AngleUnit.DEGREES));

        // turret is nullable in your init, guard it
        if (turret != null) {
            telemetry.addData("turret target angle", turret.getTargetAngle());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // always safe stop
        stopIntakeNow();
        stopShooterNow();

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
}
