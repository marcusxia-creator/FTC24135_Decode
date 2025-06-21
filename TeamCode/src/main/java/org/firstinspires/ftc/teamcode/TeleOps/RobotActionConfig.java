package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**
     * updated the parameters on 2025-02-24;
     */

    //drive chassis
    public static double powerFactor                    = 0.9;
    public static double WHEEL_DIAMETER_CM              = 9.6;                                          // unit in cm.
    public static double COUNTS_PER_MOTOR_GOBILDA_312   = 537.7;
    public static double GEAR_RATIO                     = 1.5;


    //Intake Configure
    public static double intake_Slide_Extension         = 0.25;                                         // range(0.04 - 0.29)
    public static double intake_Slide_Retract           = 0;                                         // 0.01 = 3deg rotation of gobilda servo
    public static double intake_slide_Retract_Set       = 0.06;
    public static double intake_Slide_Extension_Wait    = 2;                                         // FOR AUTO MODE ONLY

    public static double intake_Rotation_Mid            = 0.525;                                         // range(0-1, 0.46 at the middle for installation

    /**Arm range(0-0.43, lowest :0.43, fully back for transfer:0.14 )*/
    /** Todo Intake Arm installation poisition ?? and value??*/
    public static double intake_Arm_Pick                = 0.5;
    public static double intake_Arm_Grab                = 0.58;
    public static double intake_Arm_Idle                = 0.24;
    public static double intake_Arm_Transfer            = 0;
    public static double intake_Arm_Side_Drop           = 0.2;

    /**Intake Wrist range(0-1, lowest :0, fully back for transfer:1 )*/
    /** Todo Intake Wrist installation position ?? and value??*/
    public static double intake_Wrist_highbasketpause   = 0.51;
    public static double intake_Wrist_Idle              = 0.05;
    public static double intake_Wrist_Grab              = 0.02;
    public static double intake_Wrist_Transfer          = 0.54;
    public static double intake_Wrist_Side_Drop         = 0.24;

    /**Intake Claw range(0-0.27 lowest :0, fully close 0.27 )*/
    /** Todo Intake Claw installation position ?? and value??*/
    public static double intake_Claw_Open               = 0.33;                                          //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0;

    /**Intake Turret range ()*/
    public static double intake_Turret_Mid              = 0.31;
    public static double intake_Turret_Side_Drop        = 0;

    /** Deposit slide position Config */
    public static int deposit_Slide_Down_Pos            = 10;
    public static int deposit_Slide_Highbar_Pos         = 750;
    /** Todo Deposit slide High bar Score position ?? and value??*/
    public static int deposit_Slide_Highbar_Score_Pos   = 1100;
    public static int deposit_Slide_Pick_Rear_Pos       = 400;
    public static int deposit_Slide_Highbasket_Pos      = 1545;                                          //unit in mm, highbasket point

    /**Deposit Wrist  range(0-0.7, lowest :0)*/
    /** Todo Deposit Wrist installation poisition ?? and value??*/
    public static double deposit_Wrist_Dump             = 0.6;                                         //range(0.22-0.64), installation position at 90 deg0
    public static double deposit_Wrist_Transfer         = 0.05;                                         // 176 deg ~ 0.003 is 1 deg
    public static double deposit_Wrist_Hook             = 0.53;                                         // 0.003 is 1 deg
    public static double deposit_Wrist_Flat_Pos         = 0.43;                                         // FOR Deposit Hook Flat and AUTO MODE
    public static double deposit_Wrist_Pick             = 0.22;

    /**Deposit_Arm  range(0 - 0.9)*/
    /** Todo Deposit Arm installation poisition ?? and value??*/
    public static double deposit_Arm_Dump               = 0.41;
    public static double deposit_Arm_Dump_Prep          = 0.34;
    public static double deposit_Arm_Transfer           = 0.23;
    public static double deposit_Arm_Pick               = 1;
    public static double deposit_Arm_Hook               = 0;
    public static double deposit_Arm_hang_Pos           = 0.43;

    /**Deposit_Claw  range(0-0.4, lowest :0., fully open: 0.36 )*/
    /** Todo Deposit Claw installation position ?? and value??*/
    public static double deposit_Claw_Open              = 0.36;
    public static double deposit_Claw_Close             = 0;
    public static double deposit_Claw_Close_Loose       = deposit_Claw_Close+0.02;

    /**TIME CONFIGURATION - DEPOSIT*/
    public static double dumpTime                       = 0.15;                                         // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 0.4;                                         // wait then retract slide
    public static double postDumpTime                   = 0.5;
    public static double dropTime                       = 2;                                          // wait for deposit arm to drop position and open claw.
    public static double pickTime                       = 2;                                          // NOT IN USE NOW. when detect specimen and wait to close deposit claw.
    public static double transferTime                   = 0.8;                                         // sample transfer time for close deposit claw including time of waiting the slide back, arm and wrist back to transfer

    public static double VS_Retraction_Timer            = 2;

    /**TIME CONFIGURATION - INTAKE*/
    public static double intakeSlideExtendTime          = 0.5;                                          // intake slide extension time
    public static double intakeSlideRetractSetPointTime = 0.2;                                          // intake slide retract back to 2/3 position time.
    public static double intakeWristRotationTime        = 0.2;                                          // intake wrist rotation from pick to transfter time
    public static double intakeTurretTurnTime           = 0.1;

    /**TIME CONFIGURATION - GENERAL*/
    public static double DEBOUNCE_THRESHOLD             = 0.2;                                         // debounce_Threshold
    public static double waitTime                       = 0.3;                                          // general wait time
    public static long lastPressedTime                  = 2;                                            // limitSwitch timer
    public static final long debounceDelay              = 2;                                          // unit in ms, limitSwitch debouncer in ms
    public static double timeOut                        = 2;

    /**DEPOSIT vertical slide power*/
    public static double deposit_Slide_UpLiftPower      = 1.0;                                          // slides up power
    public static double deposit_Slide_DownLiftPower    = 0.8;                                          // slides down power

    //distnace
    public static double backwardDist                   = -25;                                           // distance in cm, retract distance after hook specimen on high bar

    /**TeleOp Speed Control accel factor.*/
    public static double accel_Slowness                 = 0.25;                                         // drive control speed accel factor (0.2 -1.0); 0.2 is the slowest, 1.0 is the fastest
    public static double decel_Slowness                 = 0.5;                                          // drive control speed deaccel factor (0.2 -1.0); 0.2 is the slowest, 1.0 is the fastest

    public static int slideTickThreshold                = 20;                                           // vertical slide threshold value to determine if the slide is at position
}