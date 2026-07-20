package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

import java.util.List;

public class TeleOpTelemetryManager {

    private final Telemetry telemetry;

    private int counter;
    private int updateEveryLoops = 5;

    public TeleOpTelemetryManager(
            Telemetry telemetry
    ) {
        this.telemetry = telemetry;
    }

    public void setUpdateEveryLoops(int loops) {
        updateEveryLoops =
                Math.max(1, loops);
    }

    /**
     * Returns true only when telemetry should be sent.
     */
    private boolean shouldUpdate() {
        counter++;

        if (counter >= updateEveryLoops) {
            counter = 0;
            return true;
        }

        return false;
    }

    /**
     * Full competition telemetry.
     *
     * Cached values supplied by the main TeleOp:
     *
     * - currentPose
     * - currentZone
     * - currentDistance
     * - currentTx
     * - batteryVoltage
     * - shooterRPM
     * - shooterTargetRPM
     *
     * This method does not recalculate Pinpoint, Limelight,
     * turret geometry, shooter PID or shooter velocity.
     */
    public void update(
            BasicTeleOp_RED_ALLIANCE.Alliance alliance,
            BasicTeleOp_RED_ALLIANCE.RobotActionState requestedActionState,
            BasicTeleOp_RED_ALLIANCE.RobotActionState activeActionState,
            List<String> switchTickLog,
            ColorDetection colorDetection,
            FSMIntake fsmIntake,
            FSMShooter fsmShooter,
            SpindexerUpd spindexer,
            RobotHardware robot,
            ShooterPowerCalculator shooterPowerCalculator,
            TurretUpd turret,
            double loopHz,
            Pose2D currentPose,
            int currentZone,
            double currentDistance,
            double currentTx,
            double batteryVoltage,
            double shooterRPM,
            int shooterTargetRPM
    ) {
        if (!shouldUpdate()) {
            return;
        }

        addGeneralTelemetry(
                alliance,
                loopHz,
                batteryVoltage
        );

        addRobotStateTelemetry(
                requestedActionState,
                activeActionState,
                fsmIntake,
                fsmShooter
        );

        addSpindexerTelemetry(
                colorDetection,
                spindexer,
                robot
        );

        addShooterTelemetry(
                fsmShooter,
                robot,
                currentDistance,
                currentZone,
                shooterRPM,
                shooterTargetRPM
        );

        addPoseTelemetry(
                currentPose
        );

        addTurretTelemetry(
                switchTickLog,
                fsmShooter,
                turret,
                robot
        );

        addLimelightTelemetry(
                currentTx,
                fsmShooter
        );

        telemetry.update();
    }

    /**
     * Simplified competition telemetry.
     */
    public void updateSimplified(
            BasicTeleOp_RED_ALLIANCE.Alliance alliance,
            BasicTeleOp_RED_ALLIANCE.RobotActionState requestedActionState,
            BasicTeleOp_RED_ALLIANCE.RobotActionState activeActionState,
            FSMIntake fsmIntake,
            FSMShooter fsmShooter,
            SpindexerUpd spindexer,
            double loopHz,
            int currentZone,
            double currentDistance,
            double currentTx,
            double batteryVoltage,
            double shooterRPM,
            int shooterTargetRPM,
            long waitMS
    ) {
        if (!shouldUpdate()) {
            return;
        }

        telemetry.addData(
                "Alliance",
                alliance
        );

        telemetry.addData(
                "Loop Hz",
                "%.1f",
                loopHz
        );

        telemetry.addData(
                "Voltage",
                "%.2f V",
                batteryVoltage
        );

        telemetry.addLine(
                "-----ROBOT STATE-----"
        );

        telemetry.addData(
                "Requested",
                requestedActionState
        );

        telemetry.addData(
                "Active",
                activeActionState
        );

        telemetry.addData(
                "Intake State",
                fsmIntake.intakeStates
        );

        telemetry.addData(
                "Shooter State",
                fsmShooter.shooterState
        );

        telemetry.addLine(
                "-----SPINDEXER-----"
        );

        telemetry.addData(
                "Slot 0",
                spindexer.slots[0]
        );

        telemetry.addData(
                "Slot 1",
                spindexer.slots[1]
        );

        telemetry.addData(
                "Slot 2",
                spindexer.slots[2]
        );

        telemetry.addLine(
                "-----SHOOTER-----"
        );

        telemetry.addData(
                "Zone",
                currentZone
        );

        telemetry.addData(
                "Distance",
                "%.1f in",
                currentDistance
        );

        telemetry.addData(
                "Target RPM",
                shooterTargetRPM
        );

        telemetry.addData(
                "Measured RPM",
                "%.0f",
                shooterRPM
        );

        telemetry.addData(
                "TX",
                Double.isFinite(currentTx)
                        ? String.format("%.2f°", currentTx)
                        : "No target"
        );

        telemetry.addData(
                "waitMS",
                Double.isFinite(waitMS)
                        ? String.format("%.2f°", waitMS)
                        : "No target"
        );

        telemetry.update();
    }

    // =========================================================
    // Full telemetry sections
    // =========================================================

    private void addGeneralTelemetry(
            BasicTeleOp_RED_ALLIANCE.Alliance alliance,
            double loopHz,
            double batteryVoltage
    ) {
        telemetry.addLine(
                "-----GENERAL-----"
        );

        telemetry.addData(
                "Alliance",
                alliance
        );

        telemetry.addData(
                "Loop Hz",
                "%.1f",
                loopHz
        );

        telemetry.addData(
                "Battery",
                "%.2f V",
                batteryVoltage
        );
    }

    private void addRobotStateTelemetry(
            BasicTeleOp_RED_ALLIANCE.RobotActionState requestedActionState,
            BasicTeleOp_RED_ALLIANCE.RobotActionState activeActionState,
            FSMIntake fsmIntake,
            FSMShooter fsmShooter
    ) {
        telemetry.addLine(
                "-----ROBOT STATE-----"
        );

        telemetry.addData(
                "Requested",
                requestedActionState
        );

        telemetry.addData(
                "Active",
                activeActionState
        );

        telemetry.addData(
                "Intake State",
                fsmIntake.intakeStates
        );

        telemetry.addData(
                "Shooter State",
                fsmShooter.shooterState
        );

        telemetry.addData(
                "Shooter Motor State",
                fsmShooter.getShooterMotorState()
        );

        telemetry.addData(
                "Turret State",
                fsmShooter.getTurretState()
        );

        telemetry.addData(
                "Intake Safe",
                fsmIntake.canExit()
        );

        telemetry.addData(
                "Shooter Safe",
                fsmShooter.canExit()
        );
    }

    private void addSpindexerTelemetry(
            ColorDetection colorDetection,
            SpindexerUpd spindexer,
            RobotHardware robot
    ) {
        telemetry.addLine(
                "-----SPINDEXER-----"
        );

        /*
         * This is still a hardware read. Because telemetry is
         * throttled, it happens once every updateEveryLoops.
         *
         * If FSMIntake already caches this distance, replace it
         * with that cached value.
         */
        telemetry.addData(
                "Distance Sensor",
                "%.1f mm",
                robot.distanceSensor.getDistance(
                        DistanceUnit.MM
                )
        );

        telemetry.addData(
                "Stable Color",
                colorDetection.getStableColor()
        );

        telemetry.addData(
                "Sensor Values",
                spindexer.colorValue
        );

        telemetry.addData(
                "Slot 0",
                spindexer.slots[0]
        );

        telemetry.addData(
                "Slot 1",
                spindexer.slots[1]
        );

        telemetry.addData(
                "Slot 2",
                spindexer.slots[2]
        );

        telemetry.addData(
                "Current Position",
                spindexer.currentPos
        );

        telemetry.addData(
                "Current Index",
                spindexer.index
        );
    }

    private void addShooterTelemetry(
            FSMShooter fsmShooter,
            RobotHardware robot,
            double currentDistance,
            int currentZone,
            double shooterRPM,
            int shooterTargetRPM
    ) {
        telemetry.addLine(
                "-----SHOOTER-----"
        );

        telemetry.addData(
                "Distance to Goal",
                "%.1f in",
                currentDistance
        );

        telemetry.addData(
                "Shooter Zone",
                currentZone
        );

        /*
         * Do not call ShooterPowerCalculator.getPower() here.
         * That would run the PID controller a second time.
         */
        telemetry.addData(
                "Calculated Power",
                "%.3f",
                fsmShooter.getPower()
        );

        /*
         * DcMotor.getPower() returns the most recently commanded
         * power and does not recalculate the shooter controller.
         */
        telemetry.addData(
                "Commanded Power",
                "%.3f",
                robot.topShooterMotor.getPower()
        );

        telemetry.addData(
                "FSM Voltage",
                "%.2f V",
                fsmShooter.getVoltage()
        );

        telemetry.addData(
                "Target RPM",
                shooterTargetRPM
        );

        telemetry.addData(
                "Measured RPM",
                "%.0f",
                shooterRPM
        );

        double rpmError =
                shooterTargetRPM
                        - shooterRPM;

        telemetry.addData(
                "RPM Error",
                "%.0f",
                rpmError
        );

        boolean shooterAtSpeed =
                shooterTargetRPM > 0
                        && shooterRPM
                        >= shooterTargetRPM * 0.95;

        telemetry.addData(
                "At Speed",
                shooterAtSpeed
        );
    }

    private void addPoseTelemetry(
            Pose2D currentPose
    ) {
        telemetry.addLine(
                "-----POSE-----"
        );

        if (currentPose == null) {
            telemetry.addData(
                    "Pose",
                    "Unavailable"
            );

            return;
        }

        telemetry.addData(
                "Pose",
                "X: %.2f  Y: %.2f  H: %.1f",
                currentPose.getX(
                        DistanceUnit.INCH
                ),
                currentPose.getY(
                        DistanceUnit.INCH
                ),
                currentPose.getHeading(
                        AngleUnit.DEGREES
                )
        );
    }

    private void addTurretTelemetry(
            List<String> switchTickLog,
            FSMShooter fsmShooter,
            TurretUpd turret,
            RobotHardware robot
    ) {
        telemetry.addLine(
                "-----TURRET-----"
        );

        telemetry.addData(
                "Snapshot Valid",
                turret.isSensorSnapshotValid()
        );

        telemetry.addData(
                "Limit Switch Pressed",
                turret.isLimitPressed()
        );

        telemetry.addData(
                "Limit Switch Log",
                switchTickLog
        );

        telemetry.addData(
                "Goal Pose",
                turret.getGoalPose()
        );

        /*
         * These TurretUpd getters return values from the current
         * loop snapshot. They do not repeat Pinpoint or encoder
         * reads.
         */
        telemetry.addData(
                "Target Angle",
                "%.2f°",
                turret.getTargetAngle()
        );

        telemetry.addData(
                "Drive Angle",
                "%.2f°",
                turret.getTurretDriveAngle()
        );

        telemetry.addData(
                "Motor Angle",
                "%.2f°",
                turret.getTurretMotorAngle()
        );

        telemetry.addData(
                "Base Target Tick",
                turret.getTargetTick()
        );

        telemetry.addData(
                "Current Tick",
                turret.getCurrentTick()
        );

        telemetry.addData(
                "Auto End Tick",
                PoseStorage.turretEndTick
        );

        telemetry.addData(
                "Turret Offset Tick",
                turret.getTurretOffsetTick()
        );

        telemetry.addData(
                "FSM Turret State",
                fsmShooter.getTurretState()
        );

        telemetry.addData(
                "Profile Position",
                "%.1f",
                turret.getProfilePositionTicks()
        );

        telemetry.addData(
                "Profile Velocity",
                "%.1f ticks/s",
                turret.getProfileVelocityTicksPerSecond()
        );

        telemetry.addData(
                "Profile Acceleration",
                "%.1f ticks/s²",
                turret.getProfileAcceleration()
        );

        telemetry.addData(
                "PID Output",
                "%.4f",
                turret.getPIDOutput()
        );

        telemetry.addData(
                "Feedforward Output",
                "%.4f",
                turret.getFeedforwardOutput()
        );

        telemetry.addData(
                "Final Motor Output",
                "%.4f",
                turret.getMotorOutput()
        );

        /*
         * getPower() returns the last commanded motor power.
         */
        telemetry.addData(
                "Motor Power",
                "%.4f",
                robot.turretMotor.getPower()
        );
    }

    private void addLimelightTelemetry(
            double currentTx,
            FSMShooter fsmShooter
    ) {
        telemetry.addLine(
                "-----LIMELIGHT-----"
        );

        telemetry.addData(
                "Has Target",
                fsmShooter.hasLimelightTarget()
        );

        if (Double.isFinite(currentTx)) {
            telemetry.addData(
                    "Filtered TX",
                    "%.2f°",
                    currentTx
            );
        } else {
            telemetry.addData(
                    "Filtered TX",
                    "No target"
            );
        }
    }
}