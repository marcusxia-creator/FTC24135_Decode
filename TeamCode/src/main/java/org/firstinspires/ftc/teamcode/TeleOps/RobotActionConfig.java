package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 0.9;
    public static double WHEEL_DIAMETER_CM = 9.6;
    public static double TICKS_PER_REVOLUTION = 384.5;
    public static double GEAR_RATIO = 0.6;
    public static double TICKS_PER_CM = 29.75;

    //Intake Configure
    public static double intake_Slide_Extension         = 0.33;// range(0.1 - 0.65)
    public static double intake_Slide_Retract           = 0.03;
    public static double intake_slide_Retract_Set      = 0.1;

    public static double intake_Rotation_Mid            = 0.46; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.12; //initial arm position, range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Pick                = 0.38; //intake arm pick pos
    public static double intake_Arm_Idle                = 0.26; // intake arm lift a bit before retract
    public static double intake_Arm_Transfer            = 0.08;  // intake arm transfer pos

    public static double intake_Wrist_Retract           = 0.20; /** Needs change**/
    public static double intake_Wrist_Idle              = 0.20;
    public static double intake_Wrist_Pick              = 0.64;
    public static double intake_Wrist_Transfer          = 0.12;

    public static double intake_Claw_Open               = 0.0; //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Config
    public static int deposit_Slide_Down_Pos             = 50;   //range (0-3300), 50 to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 1000;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 3222; //highest point
    public static int deposit_Slide_Hang_Pos            = 3700;

    public static double deposit_Wrist_Dump             = 0.22; //range(0.22-0.64), 0: installation position
    public static double deposit_Wrist_Transfer         = 0.52; // 176 deg ~ 0.003 is 1 deg
    public static double deposit_Wrist_Pick             = 0.38; // 0.003 is 1 deg
    public static double deposit_Wrist_Hook             = 0.64; // 0.003 is 1 deg
    public static double deposit_Wrist_Flat_Pos         = 0.3;


    public static double deposit_Arm_Pick               = 1; // 0 is pick position.
    public static double deposit_Arm_Dump               = 0.7; // range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Transfer           = 0.05; // 0 is rest position.
    public static double deposit_Arm_hang_Pos           = 0.25;  // hang position
    public static double deposit_Arm_Hook               = 0.95;  // deposit arm hook position
    public static double deposit_Arm_Dump_Prep          = 0.4;  // deposit arm hook position


    public static double deposit_Claw_Open              = 0.36;  //
    public static double deposit_Claw_Close             = 0.08;

    //COLOR VALUE
    public static float hsvValues[]                     = {0F,0F,0F}; // set color sensor value

    //TIME CONFIGURATION - DEPOSIT
    public static double dumpTime                       = 0.25; // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 0.25; // wait then retract slide

    public static double postDumpTime                   = dumpTime+0.25;
    public static double dropTime                       = 0.4; // wait for deposit arm to drop position and open claw.
    public static double pickTime                       = 0.5;  // when detect specimen and wait to close deposit claw.
    public static double waitTime                       = 0.2;  // general wait time
    public static double transferTime                   = 0.6;   // sample transfer time for close deposit claw
    public static double DEBOUNCE_THRESHOLD             = 0.25; // debounce_Threshold
    //TIME CONFIGURATION - INTAKE
    public static double intakeSlideExtendTime          = 0.9; // intake slide extension time
    public static double intakeWristRotationTime          = 0.4; // intake slide extension time

    //DEPOSIT vertical slide power
    public static double deposit_Slide_UpLiftPower      = 1.0;  //slides power
    public static double deposit_Slide_DownLiftPower    = 0.7;  //slides power

    public static double backwardDist                   =-90;
    public static double strafeDist                     = 130;

    public static double slowness                       =0.25;

}
