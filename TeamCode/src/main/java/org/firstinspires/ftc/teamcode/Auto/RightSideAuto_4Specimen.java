
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto_4Specimen_gw_test", group="org.firstinspires.ftc.teamcode.Auto")
@Disabled
@Config
public class RightSideAuto_4Specimen extends LinearOpMode {

    public static double highbar_x_coordinate = PointToDrive.highbar_x_coordinate;
    public static double highbar_y_coordinate = PointToDrive.highbar_y_coordinate;
    public static double highbar_x_coordinate2 = PointToDrive.highbar_x_coordinate2;
    public static double highbar_x_coordinate3 = PointToDrive.highbar_x_coordinate3;
    public static double specimen_pickup_x_coordinate = PointToDrive.specimen_pickup_x_coordinate;
    public static double specimen_pickup_y_coordinate = PointToDrive.specimen_pickup_y_coordinate;
    public static double first_sample_pickup_x_coordinate = PointToDrive.first_Color_sample_pickup_x_coordinate;
    public static double first_sample_pickup_y_coordinate = PointToDrive.first_Color_sample_pickup_y_coordinate;
    public static double second_sample_pickup_x_coordinate = PointToDrive.second_Color_sample_pickup_x_coordinate;
    public static double second_sample_pickup_y_coordinate = PointToDrive.second_Color_sample_pickup_y_coordinate;
    public static double clawOpenTimer = 0.2;
    public static double waitTimer = 0.25;
    public static double vSlideWaitTimer = 0.5;
    public static double hSlideWaitTimer =  0.35;
    public static double wristWaitTimer = 0.25;
    private long limitSwitchPressStartTime;

    private ElapsedTime timer = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    VerticalSlide vSlides;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot.init(hardwareMap);
        vSlides = new VerticalSlide(robot);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_highbasketpause);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {depositSysScoring();})
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                //Hook 1st specimen
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                //flat the wrist
                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                //wait 0.1 second
                .waitSeconds(0.1)
                /** Move to 1st sample on ground - while deposit retracting */
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate, first_sample_pickup_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{depositSysRetract();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75,()->{intakeSamplePick();})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    vSlides.Slides_Stop();
                })
                .addTemporalMarker(0,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})     //Grab sample
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{executeTransfer();})                                        //transfer sample
                .UNSTABLE_addTemporalMarkerOffset(1.0,()->{intakeSamplePick();})                                       //extend intake
                .UNSTABLE_addTemporalMarkerOffset(1.0,()->{depositHookPosition();})                                    //Rotate deposit arm to dump sample
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})  //after deposit arm to position and deposit claw to open to drop
                .UNSTABLE_addTemporalMarkerOffset(1.35,()->{depositTransferPosition();})
                .lineToLinearHeading(new Pose2d(second_sample_pickup_x_coordinate, second_sample_pickup_y_coordinate,Math.toRadians(90)))  // move robot to 2nd sample on ground
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{executeTransfer();})
                .UNSTABLE_addTemporalMarkerOffset(1.0,()->{depositHookPosition();})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})  //after deposit arm to position and deposit claw to open to drop
                .UNSTABLE_addTemporalMarkerOffset(1.3,()->{depositTransferPosition();})
                /// Move to Specimen Pick up - 2nd specimen
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intakeSpecimenPickReady();})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{ drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{intakeSpecimenPick();})
                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{executeTransfer();})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{depositSysScoring();})
                ///Score the 2nd specimen
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .addTemporalMarker(0,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);})
                .waitSeconds(0.1)
                ///move to specimen Pick up - 3rd specimen
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{depositSysRetract();})
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intakeSpecimenPickReady();})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    vSlides.Slides_Stop();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{intakeSpecimenPick();})
                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{executeTransfer();})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{depositSysScoring();})
                ///score the 3rd specimen
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .addTemporalMarker(0,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);})
                .waitSeconds(0.1)
                ///move to specimen Pick up - 4th specimen
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{depositSysRetract();})
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intakeSpecimenPickReady();})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    vSlides.Slides_Stop();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{intakeSpecimenPick();})
                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{executeTransfer();})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{depositSysScoring();})
                ///score the 4th specimen
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .addTemporalMarker(0,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);})
                .waitSeconds(0.1)
                ///move to specimen Pick up - ob zone
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{depositSysRetract();})
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intakeSpecimenPickReady();})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    vSlides.Slides_Stop();
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{intakeSpecimenPick();})
                .build();

        waitForStart();
        //set the timer
        timer.reset();
        long currentTime = System.currentTimeMillis();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq2);

        /// update the currentPose
        PoseStorage.currentPose = drive.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("Slides Left Position: ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Slides Right Position: ", robot.liftMotorRight.getCurrentPosition());
            telemetry.update();
        }
    }

    /** Slides subclass for vertical slides moving helper method.*/
    class VerticalSlide {
        RobotHardware robot;
        private VerticalSlide(RobotHardware robot){
            this.robot = robot;
        }
        private void Slides_Stop() {
            robot.liftMotorLeft.setPower(0);
            robot.liftMotorRight.setPower(0);
        }

        private void slidesMove(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        private void slidesMoveDown(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);

            long startTime = System.currentTimeMillis();  // Track start time
            long timeout = 2500;  // Set a timeout of 5 seconds to avoid getting stuck

            while (opModeIsActive() && (robot.liftMotorLeft.isBusy() || robot.liftMotorRight.isBusy())) {
                if (isLimitSwitchPressedFor200ms()) {  // Pass actual time
                    Slides_Stop();
                    break;
                }
            }
        }
    }

    /**HELPER METHODS*/
    /**
     * Returns true only if the limit switch has been pressed continuously for at least 200ms.
     */

    private boolean lSisPressed() {
        limitSwitchPressStartTime = 0;
        return robot.limitSwitch.getState();  // Invert if switch is normally open
    }
    private boolean isLimitSwitchPressedFor200ms() {
        if (lSisPressed()) {
            // If the switch is pressed and we haven't recorded the start time yet,
            // record the current time.
            if (limitSwitchPressStartTime == 0) {
                limitSwitchPressStartTime = System.currentTimeMillis();
            }
            // Check if 200ms have elapsed.
            if (System.currentTimeMillis() - limitSwitchPressStartTime >= 200) {
                return true;
            } else {
                // If the switch is not pressed, reset the timer.
                limitSwitchPressStartTime = 0;
            }
        }
        return false;
    }

    //depositTransferPosition
    private void depositTransferPosition(){
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
    }
    //depositHookPosition
    private void depositHookPosition(){
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
    }

    //DEPOSIT SYSTEM Action Helper
    public void depositSysRetract() {
        depositTransferPosition();
        vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8);
    }

    ///VERTICAL SLIDES and deposit arm scoring position
    public void depositSysScoring(){
        vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 1);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
        depositHookPosition();
    }
    /// intake Specimen Pick Ready position
    public void intakeSpecimenPickReady(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
    }

    /// intake Specimen Pick position
    public void intakeSpecimenPick(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
    }

    /// intake Specimen Pick position
    public void intakeSamplePick() {
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
    }

    ///Transfer action
    private void executeTransfer(){
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);

        waitFor(0.25);                // 0.25 secs;
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        waitFor(0.5);                // 0.25 secs;
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        waitFor(0.15);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        waitFor(0.1);
    }

    private void waitFor(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            // Optionally call idle() or update telemetry here.
        }
    }
}