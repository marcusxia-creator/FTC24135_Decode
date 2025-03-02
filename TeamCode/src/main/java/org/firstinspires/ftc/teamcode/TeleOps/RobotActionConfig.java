package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**
     * updated the parameters on 2025-02-24;
     */

    //drive chassis
    public static double powerFactor                    = 1;
    public static double WHEEL_DIAMETER_CM              = 9.6;                                          // unit in cm.
    public static double COUNTS_PER_MOTOR_GOBILDA_312   = 537.7;
    public static double GEAR_RATIO                     = 1.5;
    public static final double COUNTS_PER_MOTOR_GOBILDA_435    = 384.5;
    static final double SlidePullyCircummerance         = Math.PI*38.2;                                 // unit in mm
    public static final int TICKS_PER_MM_Slides         = (int) (COUNTS_PER_MOTOR_GOBILDA_435 / SlidePullyCircummerance); // tick per mm


    //Intake Configure
    public static double intake_Slide_Extension         = 0.29;                                         // range(0.04 - 0.29)
    public static double intake_Slide_Retract           = 0.04;                                         // 0.01 = 3deg rotation of gobilda servo
    public static double intake_slide_Retract_Set       = 0.07;
    public static double intake_Slide_Extension_Wait    = 0.16;                                         // FOR AUTO MODE ONLY

    public static double intake_Rotation_Mid            = 0.46;                                         // range(0-1, 0.46 at the middle for installation

    /**Arm range(0-0.43, lowest :0.43, fully back for transfer:0.14 )*/
    /** Todo Intake Arm installation poisition ?? and value??*/
    public static double intake_Arm_Initial             = 0.12;                                         //Arm tilted outwards slightly 0.12.
    public static double intake_Arm_Left_Pick           = 0.42; //intake arm pick pos
    public static double intake_Arm_Right_Pick          = intake_Arm_Left_Pick + 0.02;                  //0.02 is the offset for left Arm gear

    public static double intake_Arm_Left_Grab           = 0.43;                                         //grab position is the lowest
    public static double intake_Arm_Right_Grab          = intake_Arm_Left_Grab + 0.02;
    public static double intake_Arm_Left_Idle           = 0.26;                                         // intake arm lift a bit before retract
    public static double intake_Arm_Right_Idle          = intake_Arm_Left_Idle + 0.02;
    public static double intake_Arm_Left_Transfer       = 0.14;
    public static double intake_Arm_Right_Transfer      = intake_Arm_Left_Transfer + 0.02;              // intake arm transfer position
    public static double intake_Arm_Wait                = 0.4;                                          // FOR AUTO MODE ONLY

    /**Intake Wrist range(0-1, lowest :0, fully back for transfer:1 )*/
    /** Todo Intake Wrist installation poisition ?? and value??*/
    public static double intake_Wrist_highbasketpause   = 0.9;                                          /** upright when high basket  **/
    public static double intake_Wrist_Idle              = 0.20;                                         /** for specimen pick ready  **/
    public static double intake_Wrist_Pick              = 0.0;                                            /** new servo changed this to 0 for pick **/
    public static double intake_Wrist_Transfer          = 1.0;                                            // Axon servo - 0-1 = 0-180deg. 0.01 = 1.8 deg

    /**Intake Claw range(0-0.27 lowest :0, fully close 0.27 )*/
    /** Todo Intake Claw installation poisition ?? and value??*/
    public static double intake_Claw_Open               = 0.0;                                          //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Config
    public static int deposit_Slide_Down_Pos            = 4;                                            //unit in mm, to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 240;                                          //unit in mm, slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 770;                                          //unit in mm, highbasket point
    public static int deposit_Slide_Hang_Pos            = 825;                                          //unit in mm, highest point

    /**Deposit Wrist  range(0-0.43, lowest :0.43, fully back for transfer:0.14 )*/
    /** Todo Deposit Wrist installation poisition ?? and value??*/
    public static double deposit_Wrist_Dump             = 0.22;                                         //range(0.22-0.64), installation position at 90 deg0
    public static double deposit_Wrist_Transfer         = 0.52;                                         // 176 deg ~ 0.003 is 1 deg
    public static double deposit_Wrist_Pick             = 0.38;                                         // 0.003 is 1 deg
    public static double deposit_Wrist_Hook             = 0.64;                                         // 0.003 is 1 deg
    public static double deposit_Wrist_Flat_Pos         = 0.25;                                         // FOR Deposit Hook Flat and AUTO MODE

    /**Deposit_Arm  range(0.4 - 0.95, lowest : , fully back for transfer:0.14 )*/
    /** Todo Deposit Arm installation poisition ?? and value??*/
    public static double deposit_Arm_Pick               = 1;                                            // Installation position: 1 full parallel to ground facing out..
    public static double deposit_Arm_Dump               = 0.7;                                          // Axon servo -  0-180deg with range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Dump_Prep          = 0.4;                                          // deposit arm dump pre-ready position.
    public static double deposit_Arm_Transfer           = 0.05;                                         //
    public static double deposit_Arm_hang_Pos           = 0.25;                                         // hang position
    public static double deposit_Arm_Hook               = 0.95;                                         // deposit arm hook position


    /**Deposit_Claw  range(0-0.4, lowest :0., fully open: 0.36 )*/
    /** Todo Deposit Claw installation poisition ?? and value??*/
    public static double deposit_Claw_Open              = 0.36;                                         //deposit claw open position
    public static double deposit_Claw_Close             = 0.08;                                         //deposit claw close position

    //COLOR VALUE
    public static float hsvValues[]                     = {0F,0F,0F};                                   // set color sensor value

    //TIME CONFIGURATION - DEPOSIT
    public static double dumpTime                       = 0.25;                                         // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 0.25;                                         // wait then retract slide
    public static double postDumpTime                   = dumpTime+0.25;
    public static double dropTime                       = 0.4;                                          // wait for deposit arm to drop position and open claw.
    public static double pickTime                       = 0.5;                                          // NOT IN USE NOW. when detect specimen and wait to close deposit claw.
    public static double transferTime                   = 1.15;                                         // sample transfer time for close deposit claw including time of waiting the slide back, arm and wrist back to transfer

    //TIME CONFIGURATION - INTAKE
    public static double intakeSlideExtendTime          = 0.4;                                          // intake slide extension time
    public static double intakeSlideRetractSetPointTime = 0.3;                                          // intake slide retract back to 2/3 position time.
    public static double intakeWristRotationTime        = 0.4;                                          // intake wrist rotation from pick to transfter time

    //TIME CONFIGURATION - GENERAL
    public static double DEBOUNCE_THRESHOLD             = 0.25;                                         // debounce_Threshold
    public static double waitTime                       = 0.2;                                          // general wait time
    public static long lastPressedTime                  = 0;                                            // limitSwitch timer
    public static final long debounceDelay              = 100;                                          // unit in ms, limitSwitch debouncer in ms
    public static double timeOut                        = 1.5;

    //DEPOSIT vertical slide power
    public static double deposit_Slide_UpLiftPower      = 1.0;                                          // slides up power
    public static double deposit_Slide_DownLiftPower    = 0.7;                                          // slides down power

    //distnace
    public static double backwardDist                   = -20;                                           // distance in cm, retract distance after hook specimen on high bar

    //TeleOp Speed Control accel factor.
    public static double accel_Slowness                 = 0.25;                                         // drive control speed accel factor (0.2 -1.0); 0.2 is the slowest, 1.0 is the fastest
    public static double decel_Slowness                 = 0.6;                                          // drive control speed deaccel factor (0.2 -1.0); 0.2 is the slowest, 1.0 is the fastest

    public static int slideTickThreshold                = 30;                                           // vertical slide threshold value to determine if the slide is at position
}