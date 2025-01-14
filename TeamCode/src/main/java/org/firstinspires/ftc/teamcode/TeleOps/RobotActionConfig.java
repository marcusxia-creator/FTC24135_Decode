package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 1;

    //Intake Configure
    public static double intake_Slide_Extension         = 0.33;// range(0.1 - 0.65)
    public static double intake_Slide_Retract           = 0.0;

    public static double intake_Rotation_Mid            = 0.46; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.12; //initial arm position, range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Pick                = 0.41; //intake arm pick pos
    public static double intake_Arm_Idle                = 0.25; // intake arm lift a bit before retract
    public static double intake_Arm_Transfer            = 0.1;  // intake arm transfer pos

    public static double intake_Wrist_Initial           = 0.0; /** Needs change**/
    public static double intake_Wrist_Idle              = 0.1; /** Needs change**/
    public static double intake_Wrist_Pick              = 0.64; /** Needs change**/
    public static double intake_Wrist_Transfer          = 0.11; /** Needs change**/

    public static double intake_Claw_Open               = 0.0; //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Config
    public static int deposit_Slide_Down_Pos             = 50;   //range (0-3300), 50 to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 1050;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 3222; //highest point
    public static int deposit_Slide_Hang_Pos            = 3525;

    public static double deposit_Wrist_Dump             = 0.22; //range(0-1), 0: installation position
    public static double deposit_Wrist_Transfer         = 0.53; // 0.06 is rest position.
    public static double deposit_Wrist_Pick             = 0.38; // 0.06 is rest position.
    public static double deposit_Wrist_Hook             = 0.64; // 0.06 is rest position.
    public static double deposit_Wrist_Flat_Pos         = 0.3;


    public static double deposit_Arm_Pick               = 1; // 0 is pick position.
    public static double deposit_Arm_Dump               = 0.7; // range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Transfer           = 0.0; // 0 is rest position.
    public static double deposit_Arm_hang_Pos           = 0.8;  // hang position
    public static double deposit_Arm_Hook               = 0.9;  // deposit arm hook position

    public static double deposit_Claw_Open              = 0.36;  //
    public static double deposit_Claw_Close             = 0.08;

    public static float hsvValues[]                     = {0F,0F,0F}; // set color sensor value

    public static double dumpTime                       = 0.25; // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 0.25; // wait then retract slide

    public static double postDumpTime                   = dumpTime+0.25;
    public static double transferTime                   = 0.5; // wait for transfer time then open intake claw.
    public static double hookTime                       = 0.3; // wait then release deposit claw.


    public static double deposit_Slide_UpLiftPower      = 1.0;  //slides power
    public static double deposit_Slide_DownLiftPower    = 0.7;  //slides power
    public static double DEBOUNCE_THRESHOLD             =0.25; // debounce_Threshold

}
