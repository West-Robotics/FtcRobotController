package org.firstinspires.ftc.teamcode.seventh.robot.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static int LEFT_REGION_X = 0;
    public static int LEFT_REGION_Y = 0;
    public static int LEFT_REGION_WIDTH = 0;
    public static int LEFT_REGION_HEIGHT = 0;

    public static int MIDDLE_REGION_X = 0;
    public static int MIDDLE_REGION_Y = 0;
    public static int MIDDLE_REGION_WIDTH = 0;
    public static int MIDDLE_REGION_HEIGHT = 0;

    public static int RIGHT_REGION_X = 0;
    public static int RIGHT_REGION_Y = 0;
    public static int RIGHT_REGION_WIDTH = 0;
    public static int RIGHT_REGION_HEIGHT = 0;

    public static double HEADING_P = 0.01;
    public static double HEADING_D = 0.0;
    public static double LIFT_P = 0.03;
    public static double LIFT_I = 0.0;
    public static double LIFT_D = 0.0;
    public static double LIFT_F = 0.0;
    // 16/24 ratio * 0.03937 * mm to inch * 111.715 mm spool circum / 103.8 ppr, in inches
    public static double LIFT_DISTANCE_PER_PULSE = 0.6666*0.03937*111.715/103.8;
    public static double LIFT_MAX = 11.3;
    public static double LIFT_MIN = -0.2;
    public static double INTERMEDIARY_ZONE_1 = 0.3;
    public static double INTERMEDIARY_ZONE_2 = 0;
    // 0.79 -> 0.7 -> 0.38
    public static double PIVOT_INTAKE = 0.49;
    public static double PIVOT_INTERMEDIARY = 0.44;
    public static double PIVOT_INTERMEDIARY_2 = 0.55;
    public static double PIVOT_OUTTAKE = 0.0;
    public static double FINGER_L_OPEN = 0.45;
    public static double FINGER_L_CLOSE = 0.39;
    public static double FINGER_R_OPEN = 0.60;
    public static double FINGER_R_CLOSE = 0.68;

    // outer roller auto stack angle for servo in degrees
    public static double STACK_5 = 1;
    public static double STACK_4 = 0.9;
    public static double STACK_3 = 0.7;
    public static double STACK_2 = 0.6;
    public static double STACK_1 = 0.5;

    public static boolean AUTO = false;
}
