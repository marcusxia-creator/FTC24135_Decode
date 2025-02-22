package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**
     * updated the parameters on 2025-01-29;
     */

    //drive chassis
    public static double powerFactor                    = 1;
    public static double WHEEL_DIAMETER_CM              = 9.6;
    public static double TICKS_PER_REVOLUTION           = 537.7;
    public static double GEAR_RATIO                     = 1.5;
    public static double TICKS_PER_CM                   = 56;

    //Intake Configure
    public static double intake_Slide_Extension         = 0.29; // range(0.1 - 0.65)
    public static double intake_Slide_Retract           = 0.04;
    public static double intake_slide_Retract_Set       = 0.1;
    public static double intake_Slide_Extension_Wait    = 0.16; // FOR AUTO MODE ONLY

    public static double intake_Rotation_Mid            = 0.46; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.12; //initial arm position, range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Left_Pick           = 0.42; //intake arm pick pos
    public static double intake_Arm_Right_Pick          = intake_Arm_Left_Pick + 0.02;  //0.02 is the offset for gear

    public static double intake_Arm_Left_Grab           = 0.43;
    public static double intake_Arm_Right_Grab          = intake_Arm_Left_Grab + 0.02;
    public static double intake_Arm_Left_Idle           = 0.26;
    public static double intake_Arm_Right_Idle          = intake_Arm_Left_Idle + 0.02;  // intake arm lift a bit before retract
    public static double intake_Arm_Left_Transfer       = 0.14;
    public static double intake_Arm_Right_Transfer      = intake_Arm_Left_Transfer + 0.02;  // intake arm transfer pos
    public static double intake_Arm_Wait                = 0.4;  // FOR AUTO MODE ONLY

    public static double intake_Wrist_highbasketpause   = 0.9;
    public static double intake_Wrist_Idle              = 0.20; /** for high basket,  **/
    public static double intake_Wrist_Pick              = 0;    /** new servo changed this to 1 for pick **/
    public static double intake_Wrist_Transfer          = 1;

    public static double intake_Claw_Open               = 0.0;  //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Config
    public static int deposit_Slide_Down_Pos            = 6;    //unit in mm, to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 230;  //unit in mm, slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 750;  //unit in mm, highest point
    public static int deposit_Slide_Hang_Pos            = 805;  //unit in mm
    static final double COUNTS_PER_MOTOR_GOBILDA_435    = 384.5;
    public static double TICKS_PER_MM_Slides            = COUNTS_PER_MOTOR_GOBILDA_435 / 120;

    public static double deposit_Wrist_Dump             = 0.22; //range(0.22-0.64), 0: installation position
    public static double deposit_Wrist_Transfer         = 0.52; // 176 deg ~ 0.003 is 1 deg
    public static double deposit_Wrist_Pick             = 0.38; // 0.003 is 1 deg
    public static double deposit_Wrist_Hook             = 0.64; // 0.003 is 1 deg
    public static double deposit_Wrist_Flat_Pos         = 0.30;
    public static double deposit_Wrist_Retract_Pos      = 0.25; // FOR AUTO MODE ONLY


    public static double deposit_Arm_Pick               = 1;    // 0 is pick position.
    public static double deposit_Arm_Dump               = 0.7;  // range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Transfer           = 0.05; // 0 is rest position.
    public static double deposit_Arm_hang_Pos           = 0.25; // hang position
    public static double deposit_Arm_Hook               = 0.95; // deposit arm hook position
    public static double deposit_Arm_Dump_Prep          = 0.4;  // deposit arm hook position


    public static double deposit_Claw_Open              = 0.36; //
    public static double deposit_Claw_Close             = 0.08;

    //COLOR VALUE
    public static float hsvValues[]                     = {0F,0F,0F}; // set color sensor value

    //TIME CONFIGURATION - DEPOSIT
    public static double dumpTime                       = 0.25; // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 0.25; // wait then retract slide

    public static double postDumpTime                   = dumpTime+0.25;
    public static double dropTime                       = 0.4;  // wait for deposit arm to drop position and open claw.
    public static double pickTime                       = 0.5;  // when detect specimen and wait to close deposit claw.
    public static double waitTime                       = 0.2;  // general wait time
    public static double transferTime                   = 1.15; // sample transfer time for close deposit claw including time of waiting the slide back, arm and wrist back to transfer
    public static double DEBOUNCE_THRESHOLD             = 0.25; // debounce_Threshold
    //TIME CONFIGURATION - INTAKE
    public static double intakeSlideExtendTime          = 0.4;  // intake slide extension time
    public static double intakeSlideRetractSetPointTime = 0.3;
    public static double intakeWristRotationTime        = 0.4;  // intake slide extension time
    public static long lastPressedTime                  = 0;    //limitSwitch timer
    public static final long debounceDelay              = 100;  //limitSwitch debouncer in ms
    public static double timeOut                        = 1.5;

    //DEPOSIT vertical slide power
    public static double deposit_Slide_UpLiftPower      = 1.0;  //slides power
    public static double deposit_Slide_DownLiftPower    = 0.7;  //slides power

    public static double backwardDist                   =-20;   //distance in cm

    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.6;
}