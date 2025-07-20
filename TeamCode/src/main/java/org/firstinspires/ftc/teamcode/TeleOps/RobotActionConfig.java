package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**
     * updated the parameters on 2025-02-24;
     */

    //drive chassis
    /*\
    new auto deposit pos:
    Slide pos = 485
    Deposit arm = 0.84
    Deposit wrist = 0.05
    new auto drive pos:
    spike mark 1: 33.3, -45.7, 55.5 deg, wrist 45 deg to left
    obs drop: 40.5, -47.3, 329 deg, wrist 45 deg to left
    spike mark 2: 43.3, -45.7, 55.5 deg
    grabbing: 20.7, -47.3, 319 deg
    */
    public static double powerFactor                    = 0.9;
    public static double WHEEL_DIAMETER_CM              = 9.6;                                          // unit in cm.
    public static double COUNTS_PER_MOTOR_GOBILDA_312   = 537.7;
    public static double COUNTS_PER_MOTOR_GOBILDA_435   = 384.5;
    public static double GEAR_RATIO                     = 1.5;
    public static double SPOOL_DIAMETER                 = 35.5;                                         // unit in mm, 48 for large, 35.5 for small
    public static double SPOOL_CIRCUMFERENCE            = SPOOL_DIAMETER*Math.PI;                       //151MM for 35.5mm diameter
    public static double TICKS_PER_MM_SLIDES            = COUNTS_PER_MOTOR_GOBILDA_435/(0.66*SPOOL_CIRCUMFERENCE) ;     //3.45 tick per MM

    //Intake Configure
    public static double intake_Slide_Extension         = 0.2;                                          // range(0.04 - 0.29)
    public static double intake_Slide_Retract           = 0;                                            // 0.01 = 3deg rotation of gobilda servo
    public static double intake_slide_Retract_Set       = 0.06;
    public static double intake_Slide_Extension_Wait    = 0.1;                                          // FOR AUTO MODE ONLY

    public static double intake_Rotation_Mid            = 0.48;                                         // range(0-1, 0.5 at the middle for installation

    public static double intake_Rotation_Side_Drop      = 0.3;
    public static double intake_Rotation_ini            = intake_Rotation_Mid;
    /**Arm counter clock wise from 0 to 1 */
    /** Todo Intake Arm installation poisition ?? and value??*/
    public static double intake_Arm_Pick                = 0.48;
    public static double intake_Arm_Grab                = 0.52;                                     // 0.54 is a guess value
    public static double intake_Arm_Idle                = 0.38;                                     // 0.38 is 30 degree up from pick position - flat.
    public static double intake_Arm_Transfer            = 0.13;                                      // facing up straight
    public static double intake_Arm_Side_Drop           = 0.36;                                     // 40 degree up from pick position - flat
    public static double intake_Arm_Wait                = intake_Arm_Idle;                          //

    public static double intake_Arm_Initial             = intake_Arm_Transfer;

    /**Intake Wrist counter clock wise from 0 to 1 */
    /** Todo Intake Wrist installation position ?? and value??*/
    public static double intake_Wrist_highbasketpause   = 0.44;                                    // 5 degree away from transfer
    public static double intake_Wrist_Idle              = 0.47;
    public static double intake_Wrist_Grab              = 0.7;
    public static double intake_Wrist_Transfer          = 0.42;
    public static double intake_Wrist_Side_Drop         = 0.54;
    public static double intake_Wrist_Initial           = intake_Wrist_Idle;

    /**Intake Claw range(0-0.27 lowest :0, fully close 0.27 )*/
    /** Todo Intake Claw installation position ?? and value??*/
    public static double intake_Claw_Open               = 0.33;                                     //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0;

    /**Intake Turret range ()*/
    public static double intake_Turret_Mid              = 0.46;
    public static double intake_Turret_Side_Drop        = 0.15;

    /** Deposit slide position Config */
    public static int deposit_Slide_Down_Pos            = 2;                                        //2mm from bottom u channel
    public static int deposit_Slide_Rear_Highbar_Pos    = 303;                                      //303 mm, 303 *3.45= 1046
    public static int deposit_Slide_Highbar_Pos         = 348;                                      //348mm, 348*3.45 = 1203 ticks
    /** Todo Deposit slide High bar Score position ?? and value??*/
    public static int deposit_Slide_Highbar_Score_Pos   = 567;                                      //567mm, 567*3.45 = 1957 ticks originally - too high when testing
    public static int deposit_Slide_Pick_Rear_Pos       = 145;                                      //145 in mm - 506.25 tick
    public static int deposit_Slide_Highbasket_Pos      = 675 ;                                     //700 in mm, highbasket point (for ticks  = 720*3.45 = 2483 ticks), 48inch - 21inch = 27inches vertical distance required.
                                                                                                    // slide maximum  length  = 29 inchs ~ 736 mm and 10degree incline, so 28 inches maximum vertical distance.

    /**Deposit Wrist  clock wise direction for range(0-1, lowest :0)*/
    /** Todo Deposit Wrist installation poisition ?? and value??*/

    public static double deposit_Wrist_Dump             = 0.6;                                      //
    public static double deposit_Wrist_Transfer         = 0.08;                                      //original 0.05  ; 7/5 change to 0.09
    public static double deposit_Wrist_Hook             = 0.53;                                     //0.003 is 1 deg
    public static double deposit_Wrist_Flat_Pos         = 0.43;                                     //FOR Deposit Hook Flat and AUTO MODE
    public static double deposit_Wrist_Pick             = 0.22;
    public static double deposit_Wrist_Rear_Hook        = 0.05;
    public static double deposit_Wrist_Initial          = deposit_Wrist_Transfer;                                      //

    /**Deposit_Arm  range(0 - 0.9)*/
    /** Todo Deposit Arm installation poisition ?? and value??*/
    public static double deposit_Arm_Dump               = 0.48;
    public static double deposit_Arm_Dump_Prep          = 0.34;                                     //facing up
    public static double deposit_Arm_Transfer           = 0.23;
    public static double deposit_Arm_Pick               = 1;
    public static double deposit_Arm_Hook               = 0;                                        //completely flat facing forward.
    public static double deposit_Arm_hang_Pos           = 0.3;
    public static double deposit_Arm_Rear_Hook          = 0.84;
    public static double deposit_Arm_Initial          = deposit_Arm_Transfer;                                      //

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
    public static double transferTime                   = 0.8;                                         // sample transfer time for close deposit claw including time of waiting the slide back, arm and wrist back to transfer; 7/5 changed to 0.8 from 0.7

    public static double VS_Retraction_Timer            = 2;

    /**TIME CONFIGURATION - INTAKE*/
    public static double intakeSlideExtendTime          = 0.5;                                          // intake slide extension time
    public static double intakeSlideRetractSetPointTime = 0.2;                                          // intake slide retract back to 2/3 position time.
    public static double intakeWristRotationTime        = 0.2;                                          // intake wrist rotation from pick to transfter time
    public static double intakeTurretTurnTime           = 0.2;

    /**TIME CONFIGURATION - GENERAL*/
    public static double DEBOUNCE_THRESHOLD             = 0.2;                                         // debounce_Threshold
    public static double waitTime                       = 0.2;                                          // general wait time
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