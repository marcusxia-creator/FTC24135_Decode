package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Optional;

@Config
public class Turret {
    /*We will control the turret using input from the pinpoint
    turret control is in ShooterFMS
     */

    private final RobotHardware robot;

    private ElapsedTime pressedTimer                    = new ElapsedTime();
    public static double txToTickMultiplier             = 10;
    private final double tickToAngle                    = (0.16867469879518 * (360 / 145.1)); // reduction ratio * (angle / motor Resolution tick)
    private final double angleToTick                    = 1.0 / tickToAngle;
    private double conversionFactor                     = 39.3700787;  // ??inch to mm conversion??

    // Motion profile state
    private double profilePositionTicks                     = 0;
    private double profileVelocityTicksPerSecond            = 0;
    private double lastTime                                 = 0;
    private double commandedTick                            = 0;
    private double commandedVelocity                        = 0;
    private long previousTimeNs;
    private long currentTimeNs = System.nanoTime();
    // Motion limits - Tunable constraints (Dashboard)
    public static double maxVel                             = 1200; // ticks/sec
    public static double maxAccel                           = 10000; // ticks/sec^2

    public static double turretToleranceTicks               = 5.0;
    public static double turretVelocityToleranceTicksPerSec = 5.0;
    private int turretOffsetTick                            = 0; // offset tick after turret reset

    private double turretPIDOutput                          = 0.0;
    private double turretFeedforwardOutput                  = 0.0;
    private double turretMotorOutput                        = 0.0;
    private double turretProfileAcceleration                = 0.0;

    private boolean turretProfileInitialized                = false;
    private boolean turretProfileAtTarget                   = false;
    private double turretLSZeroTick                         = 0.0;


    //TODO - PID & Feedforward constants Tuning
    public static double kPTurret = 0.004, kITurret = 0, kDTurret = 0.0003, kSTurret = 0.0001, kVTurret = 0.0004, kATurret = 0.0004;

    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);

    // Target pose
    private final LUT<Integer, Pose2D> redTargetPose    = new LUT<Integer, Pose2D>() {{
        add(1, redCloseGoalPose);
        add(2, redFarGoalPose);
    }};

    private final LUT<Integer, Pose2D> blueTargetPose   = new LUT<Integer, Pose2D>() {{
        add(1, blueCloseGoalPose);
        add(2, blueFarGoalPose);
    }};

    private LUT<Integer, Pose2D> targetPose;

    private Pose2D goalPose;

    private PIDController pidController;

    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);



    public Turret (RobotHardware robot, boolean isRedAlliance) {
        this.robot = robot;
        pidController = new PIDController(kPTurret, kITurret, kDTurret);
        pidController.setTolerance(5.0);

        // KEEP logic but simplify
        targetPose = isRedAlliance ? redTargetPose : blueTargetPose;

        // Prevent goalPose null before updateZoneForGoalPose() is called
        goalPose = Optional.ofNullable(targetPose.get(1)).orElse(targetPose.get(2));
    }

    public int getMotorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() * angleToTick);
    }

    public double getTurretDriveAngle () {
        return -(floorMod(robot.pinpoint.getHeading(AngleUnit.DEGREES) - getTargetAngle()+180, 360)-180);
    }

    public double getTurretMotorAngle(){
        return (robot.turretMotor.getCurrentPosition() * tickToAngle);
    }

    public void driveTurretMotor(){
        //used in test telop for tuning;
        int ticks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    public void driveTurretPID(int currentTick, int targetTick) {
        int errorTicks = targetTick - currentTick;
        // Feedforward should NOT be based on absolute targetTicks (too large).
        // Use direction + error assist.
        double ff = (kSTurret *Math.signum(errorTicks)) ;
        double power = pidController.calculate(currentTick, targetTick);
        double output = power + ff;
        robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
    }

    public void driveTurretPIDF (int currentTick, int targetTick) {
        long currentTimeNs = System.nanoTime();

        double measuredDeltaTime = (currentTimeNs - previousTimeNs) / 1_000_000_000.0;
        previousTimeNs = currentTimeNs;

        if (measuredDeltaTime <= 0.0)
        {
            measuredDeltaTime = 0.01;
        }
        /*
         * Limit only the profile timestep.
         * This prevents a long loop from causing a large profile jump.
         */
        double profileDeltaTime = Math.min(measuredDeltaTime, 0.1);

        /*
         * Start the profile from the actual turret position.
         */
        if (!turretProfileInitialized) {
            profilePositionTicks = currentTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileInitialized = true;
        }
        /*
         * distance from the profile position to the target
         */
        double remainingDistanceTick = targetTick - profilePositionTicks;
        double direction = Math.signum(remainingDistanceTick);

        /*
         * Maximum velocity that still allows the turret to stop at
         * the target:
         * v = sqrt(2 * acceleration * distance)
         */
        double maximumVelocityAllowedNow  =
                Math.sqrt(2.0 * maxAccel * Math.abs(remainingDistanceTick));

        /*
         * Velocity requested by the profile.
         *
         * It cannot exceed:
         * 1. The configured maximum velocity
         * 2. The velocity from which it can still stop
         */
        double desiredVelocity = direction * Math.min(maxVel,maximumVelocityAllowedNow);

        /*
         * Limit how much velocity can change this loop.
         * velocity change = acceleration * time
         */
        double maximumVelocityChange = maxAccel * profileDeltaTime;

        double previousProfileVelocity = profileVelocityTicksPerSecond;

        profileVelocityTicksPerSecond = applyAccelerationLimit(
                profileVelocityTicksPerSecond,
                desiredVelocity,
                maximumVelocityChange
            );

        /*
         * Calculate profile acceleration for kA feedforward.
         */
        turretProfileAcceleration = ( profileVelocityTicksPerSecond - previousProfileVelocity ) / profileDeltaTime;

        /*
         * Update the profile position.
         */
        profilePositionTicks += profileVelocityTicksPerSecond * profileDeltaTime;

        /*
         *prevent the generated profile from crossing the target.
         */
        if (direction > 0 && profilePositionTicks > targetTick) {
            profilePositionTicks = targetTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;
        } else if (direction < 0 && profilePositionTicks < targetTick) {
            profilePositionTicks = targetTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;
        }

        turretPIDOutput =
                pidController.calculate(
                        currentTick,
                        profilePositionTicks
                );
        /* * Feedforward kS power */
        double staticFeedforward = 0.0;
        if (Math.abs(profileVelocityTicksPerSecond) > 1.0) {
            staticFeedforward = kSTurret * Math.signum(profileVelocityTicksPerSecond);
        }
        turretFeedforwardOutput =
                staticFeedforward
                + kVTurret * profileVelocityTicksPerSecond
                + kATurret * turretProfileAcceleration;
        /// Motor Power
        turretMotorOutput = Range.clip( turretPIDOutput  + turretFeedforwardOutput,-1.0,1.0);
        /// When completely finished, remove small unnecessary output.
        if (Math.abs(targetTick - currentTick) <= turretToleranceTicks
                && Math.abs(profileVelocityTicksPerSecond) <= turretVelocityToleranceTicksPerSec) {
            profilePositionTicks = targetTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;

            /*
             * Set zero only if your turret does not need holding power.
             */
            turretMotorOutput = 0.0;
        }
        /// drive
        robot.turretMotor.setPower(turretMotorOutput);

    }
    // Call whenever the motor is driven by something other than driveTurretPIDF()
    // (e.g. manual/locking control) so the profile resyncs to the real position
    // next time driveTurretPIDF() resumes, instead of chasing a stale setpoint.
    public void resetTurretProfile() {
        turretProfileInitialized = false;
    }

    private double applyAccelerationLimit(double profileVelocityTicksPerSecond,double desiredVelocity,double maximumChange)
    {
        double difference = desiredVelocity - profileVelocityTicksPerSecond;
        if (Math.abs(difference) <= maximumChange) {
                return desiredVelocity;
            }
        else{
               return profileVelocityTicksPerSecond +Math.signum(difference)*maximumChange;
        }
    }
    public int getTargetTick () {
        return (int)Math.round(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
    }

    public int getCurrentTick () {
        return robot.turretMotor.getCurrentPosition();
    }

    public double getTargetAngle () {
        // NEW -- Fail-safe: if goalPose somehow isn't set, don't spin
        if (goalPose == null) return robot.pinpoint.getHeading(AngleUnit.DEGREES);
        double turretYaw = THETA + robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
        double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);
        double turretcentX = robot.pinpoint.getPosX(DistanceUnit.INCH) + turretXOffSet;
        double turretcentY = robot.pinpoint.getPosY(DistanceUnit.INCH) + turretYOffSet;
        return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-turretcentY), (goalPose.getX(DistanceUnit.INCH)-turretcentX)));
    }

    // select target goal pose
    public void updateZoneForGoalPose(int zone) {
        int normalizedZone;

        if (zone <= 5) {
            normalizedZone = 1;
        }
        else {
            normalizedZone = 2;
        }

        goalPose = Optional.ofNullable(targetPose.get(normalizedZone)).orElse(targetPose.get(1));
    }

    public Pose2D getGoalPose() {
        return goalPose;
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }

    public boolean turretReset(int startingTick){
        int currentTick = robot.turretMotor.getCurrentPosition();
        if (isLimitPressed()){
            robot.turretMotor.setPower(0);
            robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretLSZeroTick = robot.turretMotor.getCurrentPosition();
            return true;
        }

        int delta = currentTick - startingTick;
        boolean nearStart = Math.abs(delta) < 435;

        double baseDir = (startingTick < 0) ? 1.0 : -1.0;
        turretOffsetTick = (int) baseDir*18;
        double dir = nearStart ? baseDir : -baseDir;
        robot.turretMotor.setPower(0.5*dir);
        return false;
    }

    public int getTurretOffsetTick() {
        return turretOffsetTick;
    }

    public boolean isLimitPressed (){
        return robot.limitSwitch.getState();
    }

    public void updatePIDFParameters(
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA
    ) {
        pidController.setPID(kP, kI, kD);

        this.kSTurret = kS;
        this.kVTurret = kV;
        this.kATurret = kA;
    }
}
