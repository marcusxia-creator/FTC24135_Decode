package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    /**drive chassis**/
    public static double drive_Power_Factor = 1;

    /**Intake Config**/
    public static double intake_Slide_Extend = 0.36;// range(0 - 0.3)
    public static double intake_Slide_Retract = 0.1;

    public static double intake_Arm_Initial = 0.12;
    public static double intake_Arm_Pick = 0.415;
    public static double intake_Arm_Transfer = 0.1;
    public static double intake_Arm_Idle = 0.25;
    //public static double intake_Arm_Change_Amount = 0.05;

    public static double intake_Wrist_Initial = 0.0;
    public static double intake_Wrist_Idle = 0.6;
    public static double intake_Wrist_Pick = 0.64;
    public static double intake_Wrist_Transfer = 0.07;

    public static double intake_Rotation_Idle = 0.46;
    public static double intake_Rotation_Steer_Amount = 0.15;

    public static double intake_Claw_Open = 0;
    public static double intake_Claw_Close = 0.27;


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

    /**Timers**/
    //Intake timer
    public static double intake_Claw_Grab_Threshold = 0.25;
    public static double intake_Slide_Retract_Threshold = 0.2;
    public static double intake_Wrist_Arm_Retract_Threshold = 0.5;
    public static double deposit_Claw_Close_Threshold = 1;
    //Debounce timer
    public static double DEBOUNCE_THRESHOLD = 0.25;

    /**Other properties**/
    //Hsv values
    public static float hsvValues[] = {0F,0F,0F};
}