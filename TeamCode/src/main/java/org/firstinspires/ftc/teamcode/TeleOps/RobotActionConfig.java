package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**drive chassis**/
    public static double drive_Power_Factor = 0.7;

    /**Intake Config**/
    public static double intake_Slide_Extend = 0.55;// range(0.3 - 0.65)
    public static double intake_Slide_Retract = 0.32; /** Zirui Changed this to 0.32 (0.3 original) for the slide to stop without slamming on the chassis **/

    public static double intake_Arm_Idle = 0.1;//0-0.56
    public static double intake_Arm_Extend = 0.05;
    public static double intake_Arm_Retract = 0.53;
    public static double intake_Arm_Change_Amount = 0.05;

    public static double intake_Wrist_Extend = 0.5; /** Needs change**/
    public static double intake_Wrist_Retract = 0; /** Needs change**/

    public static double intake_Rotation_Default = 0.49;
    public static double intake_Rotation_Steer_Amount = 0.1;

    public static double intake_Claw_Open = 0.55;
    public static double intake_Claw_Close = 0.3;


    /**Deposit Config**/
    public static int deposit_Slide_down_Pos = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Pos = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos = 2800; //slides Position Configure

    public static double deposit_Wrist_dump_Pos = 0.3;
    public static double deposit_Wrist_retract_Pos = 0.15;

    public static double deposit_Arm_dump_Pos = 0.8;
    public static double deposit_Arm_retract_Pos = 0.0;

    public static double deposit_Arm_hang_Pos = 0.8;

    public static double deposit_Arm_Highbar_Pos = 0;
    public static double deposit_Wrist_Highbar_Pos = 0;

    public static double deposit_Claw_Open = 0.11;
    public static double deposit_Claw_Close = 0.0;

    public static double dumpTime = 1.8;
    public static double retractTime = 3.2;

    public static double deposit_Slide_UpLiftPower = 0.9;  //slides power
    public static double deposit_Slide_DownLiftPower = 0.3;

    /**Other properties**/
    //Hsv values
    public static float hsvValues[] = {0F,0F,0F};

    //Debounce timer
    public static double DEBOUNCE_THRESHOLD = 0.25;
}