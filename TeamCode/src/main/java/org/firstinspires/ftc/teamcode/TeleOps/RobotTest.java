package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;



@TeleOp (name = "RobotTest", group = "org.firstinspires.ftc.teamcode")
public class RobotTest extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;

    // Example: gobilda 1150= 145.1 ticks/rev, 6000=28, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double SHOOTER_TICKS_PER_REV = 28;   // change to your motor
    public static double INTAKE_TICKS_PER_REV = 145.1;   // change to your motor
    public static double SHOOTER_RPM_CONVERSION = 60.0 / SHOOTER_TICKS_PER_REV;
    public static double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    private double shooter_rpm;
    private double intake_rpm;

    private ColorDetection colorDetection;

    private boolean colorStable;

    //
    private ElapsedTime jamTimer = new ElapsedTime();

    // Determine if Intake is jammed
    private boolean backPressed = false;


    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initPinPoint();

        colorDetection = new ColorDetection(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoposition = 0.0;
        speed = 0.0;

        robot.pushRampServo.setPosition(RAMP_RESET_POSITION);

        robot.leftGateServo.setPosition(GATEUP);
        robot.rightGateServo.setPosition(GATEUP);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorDetection.startDetection();
    }

    @Override
    public void loop() {
        // === Read velocity in ticks/sec and convert to RPM ===
        double shooter_ticksPerSec = robot.shooterMotor.getVelocity();

        shooter_rpm = shooter_ticksPerSec * SHOOTER_RPM_CONVERSION;

        double intake_ticksPerSec = robot.intakeMotor.getVelocity();

        intake_rpm = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

        boolean jammed = isJammed();

        colorDetection.updateDetection();

        if (colorDetection.isBallPresent()) {
            if (colorDetection.isColorStable()) {
                robot.leftGateServo.setPosition(GATEDOWN);
                robot.rightGateServo.setPosition(GATEDOWN);

                if (colorDetection.getStableColor() == BallColor.GREEN) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
                }
                if (colorDetection.getStableColor()== BallColor.PURPLE) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot3);
                }
            }
        }
        else {
            robot.leftGateServo.setPosition(GATEUP);
            robot.rightGateServo.setPosition(GATEUP);

        }

        if (gamepad_2.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            //servoposition = robot.pushRampServo.getPosition() + 0.01;
            //robot.pushRampServo.setPosition(Range.clip(servoposition, 0.0, 1.0));
            robot.pushRampServo.setPosition(RAMP_UP);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            //servoposition = robot.pushRampServo.getPosition() - 0.01;
            //robot.pushRampServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.pushRampServo.setPosition(RAMP_RESET_POSITION);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            double i = 130.0 / 275.0;
            servoposition = robot.spindexerServo.getPosition() + i;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 0.92));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            double i = 130.0 / 275.0;
            servoposition = robot.spindexerServo.getPosition() - i;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 0.92));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            //servoposition = robot.leftGateServo.getPosition() + 0.01;
            servoposition = GATEUP;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 0.5));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,0.5));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            //servoposition = robot.leftGateServo.getPosition() - 0.01;
            servoposition = GATEDOWN;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 0.5));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,0.5));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            speed = robot.shooterMotor.getPower() + 0.05;
            robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shooterMotor.setPower(Range.clip(speed,0.5,1.0));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            robot.shooterMotor.setPower(0);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
            speed = robot.intakeMotor.getPower() + 0.05;
            robot.intakeMotor.setPower(Range.clip(speed,0.5,1.0));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

        if (jammed) {
            robot.intakeMotor.setPower(-0.2);
            try {
                sleep(300);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            robot.intakeMotor.setPower(0.0);
        }

        if (gamepad_2.getButton(GamepadKeys.Button.BACK)) {
            if (!backPressed) {
                robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH,0, 0.0, AngleUnit.DEGREES, 0)); // reset pose (x,y,heading)
                telemetry.addLine("Pinpoint reset to (0,0,0)");
                backPressed = true; // debounce
            }
        } else {
            backPressed = false; // reset debounce when released
        }


        // Telemetry (DS + Dashboard)
        // === TELEMETRY ===
        telemetry.addLine("-------Odometry-------------------");
        double currentX = robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        double currentY = robot.pinpointDriver.getPosY(DistanceUnit.INCH);
        // Compute distance to top-right corner
        double dx = -70.0 - currentX;
        double dy = 70.0 - currentY;
        double distanceToCorner = Math.hypot(dx, dy);

// Optional: direction (heading) to corner if needed
        double headingToCorner = Math.toDegrees(Math.atan2(dy, dx));
        telemetry.addData("Pinpoint X", "%.2f in", robot.pinpointDriver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y", "%.2f in", robot.pinpointDriver.getPosY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading", "%.2f°", robot.pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distance to Corner", "%.2f in", distanceToCorner);
        telemetry.addData("Heading to Corner", "%.2f°", headingToCorner);
        telemetry.addLine("-------Intake-------------------");
        telemetry.addData("Ramp Position", robot.pushRampServo.getPosition());
        telemetry.addData("Left Gate Position", robot.leftGateServo.getPosition());
        telemetry.addData("Right Gate Position", robot.rightGateServo.getPosition());
        telemetry.addLine("----color----");
        telemetry.addData("Color Stable", colorDetection.isColorStable());
        telemetry.addData("Ball Present", colorDetection.isBallPresent());
        telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Hue", colorDetection.getHue());
        telemetry.addData("Color", colorDetection.getStableColor());
        telemetry.addLine("----Spindexer----");
        telemetry.addData("Spindexer Position", robot.spindexerServo.getPosition());
        telemetry.addLine("----Shooter----");
        telemetry.addData("Shooter Speed", robot.shooterMotor.getPower());
        telemetry.addData("Shooter Ticks/sec", "%.1f", shooter_ticksPerSec);
        telemetry.addData("Shooter RPM", "%.1f", shooter_rpm);
        telemetry.addLine("----Intake----");
        telemetry.addData("Intake Jam Status", jammed);
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());
        telemetry.addData("Intake Ticks/sec", "%.1f", intake_ticksPerSec);
        telemetry.addData("Intake RPM", "%.1f", intake_rpm);

        telemetry.update();
    }

    // Button Debounce Helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    // Intake Jam Helper
    private boolean isJammed() {
        if (robot.intakeMotor.getPower() > 0.2 && intake_rpm < RobotActionConfig.intakeRPM_THRESHOLD) {
            if (jamTimer.seconds() > 0.3) return true; // jam confirmed for 0.3s
        } else {
            jamTimer.reset();
        }
        return false;
    }

}



