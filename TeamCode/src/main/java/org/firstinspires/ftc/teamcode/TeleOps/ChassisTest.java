package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/**
 * Adding Pose2D storage + turret heading from auto to teleOp
 * Turret PIDF
 ---------------------------------------------------------------------------
 * Change shooting timing values when hardware is changed for rapid shooting
 * Tuning LUT values when the hardware changes
 */
@Config
@TeleOp(name = "Chassis Test", group = "org.firstinspires.ftc.teamcode")
public class ChassisTest extends OpMode {
    /// Enum states for robot action state
    /// Enum states for alliance
    /// robot and subsystem
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private GamepadComboInput gamepadComboInput;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator

    // for time and frequency
    private long lastLoopTime = 0;
    private double loopHz = 0.0;
    private double loopTime = 0.0;

    /// ----------------------------------------------------------------
    /// For robot action state

    ///efficient telemetry
    public static double telemetryInterval;
    public ElapsedTime telemeteryTimer=new ElapsedTime();

    /// For expensive values from getter
    private int currentZone;
    private double currentDistance;
    private double currentTx;
    private Pose2D cachedPosition;
    private double shooterMeasuredRPM;
    private int shooterTargetRPM;
    private double turretDriveAngle;
    private int turretCurrentTick;
    private int turretTargetTick;
    private double turretTargetAngle;
    private double turretMotorAngle;

    /// ----------------------------------------------------------------
    @Override
    public void init() {
        /// For telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /// For robot hardware initialization
        robot = new RobotHardware(hardwareMap);
        robot.init();                       //Initialize all motors and servos
        robot.initIMU();                    //Initialize control hub IMU

        //robot.initExternalIMU();            //Initialize external IMU

        /**
         * Transfer the pose 2D from Auto Ops
         */

        ;

        /// 0. gamepad---------------------------------------------------------------
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        gamepadComboInput = new GamepadComboInput(gamepadCo1,gamepadCo2);
        /// 1. robot drive-------------------------------------------------------------

        robotDrive = new RobotDrive(robot, gamepadComboInput);
        robotDrive.Init();


        /// 4.1. power calculator for shooter------------------------------------------------------------

        /// 5. intake------------------------------------------------------------

        /// 7. alliance selection-----------------------------------------------------------

        /// 8. robot state----------------------------------------------------------


    }

    @Override
    public void loop() {
        gamepadComboInput.update();
        double intakePower = robot.intakeMotor.getPower();
        if (gamepadComboInput.getDpadLeftPressedAny()){
            intakePower += 0.1;
        } else if (gamepadComboInput.getDpadRightPressedAny()){
            intakePower -= 0.1;
        }else if (gamepadComboInput.getBPressedAny()){
            intakePower = 0;
        }
        robot.intakeMotor.setPower(Range.clip(intakePower, -1, 1));


        // =========================================================
        // 2. INPUT UPDATE (read buttons + combos)
        // =========================================================
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();


        // =========================================================
        // 3. CONTINUOUS SENSOR / HOUSEKEEPING UPDATES

        updateLoopFrequency();

        // =========================================================
        // 4. DRIVE (always responsive)
        // =========================================================
        robotDrive.DriveLoop();

        // =========================================================
        // 5. NEW! - ACTION STATE TRANSITION MANAGER (GRACEFUL)
        // =========================================================
        updateActionStateTransitions();

        // =========================================================
        // 9. TELEMETRY & Parameters
        // =========================================================

        // =========================================================================
        // runTimeTelemetry() calls telemetry.update() itself
        // — don't call it again here or telemetry.update()
        // telemetryInterval - 0.1s
        // both Driver Station and FTC Dashboard) runs throttled 0.1s refresh - 10Hz.
        // =========================================================================
        /**
         runTimeTelemetry(
         cachedPosition, currentZone, turretCurrentTick, turretTargetTick
         );
         */
        debugTelemetry();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    // =========================================================
    // Update Action State based on button presses
    // - prevent change Action State when FSM not finish
    // =========================================================
    private void updateActionStateTransitions() {

        // Decide which subsystems must be "safe" before switching

        // Example policy:
        // - Switching INTO Intaking requires shooter safe (so you don't intake while shooting)
        // - Switching INTO Shooting requires intake safe (so you don't stop intake mid-park)
        // - Switching INTO Idle requires both safe, or you can force a "stop request" first (recommended)
        // ------------------------------------------------------------
        // 1) If we are NOT safe yet, request graceful stop(s)
        //    (this will not hard-cut; it triggers each FSM's stop sequence)
        // ------------------------------------------------------------

        //TODO: if specific stop needed. uncomment the function below
        // requestGracefulstopIfNeeded - only for switching from intaking to shooting

        // ------------------------------------------------------------
        // 2) Decide if we are allowed to switch into the requested state
        // ------------------------------------------------------------
        boolean canSwitch = false;


        // ------------------------------------------------------------
        // 3) Commit switch only when safe
        // ------------------------------------------------------------
    }
    // =========================================================
    // TODO: not use, but if needed, Greacefulstop() stop the fms before naturally finish


    ///  helper functions
    ///  - LED Update

    ///  - Frequency Updates
    private void updateLoopFrequency() {
        long now = System.currentTimeMillis();
        if (lastLoopTime != 0) {
            long dtMs = now - lastLoopTime;
            loopTime=(double)dtMs/1000;
            if (dtMs > 0) {
                loopHz = 1000.0 / dtMs;
            }
        }
        lastLoopTime = now;
    }

    public void runTimeTelemetry(Pose2D cachedPosition, int zone, int turretCurrentTick, int turretTargetTick){
        //simplified telemetry for teleop
        if(telemeteryTimer.time()>telemetryInterval){
            telemetry.addData("loop frequency (Hz)", loopHz);

            telemetry.addLine("\n---ROBOT");
            telemetry.addData(
                    "Pose2D",
                    "X: %.2f, Y: %.2f, Heading: %.2f°",
                    cachedPosition.getX(DistanceUnit.INCH),
                    cachedPosition.getY(DistanceUnit.INCH),
                    cachedPosition.getHeading(AngleUnit.DEGREES)
            );
            telemetry.addData("Dist_to_Goal","%,.0f",currentDistance);

            telemetry.addLine("\n---INTAKE");

            telemetry.addLine("\n---SHOOTER");
            telemetry.addData("Shooter Zone", zone);
            telemetry.addData("Shooter Target RPM",shooterTargetRPM);
            telemetry.addData("Shooter actual RPM", shooterMeasuredRPM);

            telemetry.addLine("\n---TURRET");
            telemetry.addData("Turret error", turretCurrentTick-turretTargetTick);

            telemetry.addLine("\n---SPINDEXER");

            telemetry.update();
            telemeteryTimer.reset();
        }
    }

    //===========================================================
    // Debug telemetry Manager
    //===========================================================
    public void debugTelemetry(){
        // Full debug dump — gate it the same way as runTimeTelemetry() so switching
        // into debug mode can't push telemetry.update() every loop tick.
        if (telemeteryTimer.time() <= telemetryInterval) return;
        telemeteryTimer.reset();



        telemetry.addData("loop frequency (Hz)", loopHz);
        telemetry.addLine("-----");
        telemetry.addLine("-----");

        telemetry.addLine("-----");
        // cached value from this loop's SequenceShooterLoop() — shooterPowerAngleCalculator.getPower()
        // re-runs the shooter PID controller as a side effect, so don't call it just to display it

        telemetry.addData("Shooter Target RPM",shooterTargetRPM);
        telemetry.addData("Shooter actual RPM","%,.0f",shooterMeasuredRPM);
        telemetry.addLine("-----");

        telemetry.addData("distance to goal", "%,.0f",currentDistance);
        telemetry.addData("Shooter Zone", currentZone);
        telemetry.addLine("Turret-----------------------------------");
        telemetry.addData("turret target angle", turretTargetAngle);
        telemetry.addData("turret drive angle", turretDriveAngle);
        telemetry.addData("turret motor angle", turretMotorAngle);
        //telemetry.addData("motor PIDF coefficient", robot.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("current motor tick", turretCurrentTick);
        telemetry.addData("target motor tick", turretTargetTick);
        telemetry.addLine("-----------------------------------------");
        ///telemetry.addData("limelight output", "%,.1f",limelight.normalizedPose2D(DistanceUnit.INCH));
        telemetry.addData("limelight angle Tx", currentTx);
        telemetry.update();
    }

}
