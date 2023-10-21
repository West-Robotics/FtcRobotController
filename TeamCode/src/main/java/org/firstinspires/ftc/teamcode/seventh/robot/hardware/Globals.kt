package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.acmerobotics.dashboard.config.Config

@Config
object Globals {
    @JvmField var LEFT_REGION_X = 0
    @JvmField var LEFT_REGION_Y = 0
    @JvmField var LEFT_REGION_WIDTH = 0
    @JvmField var LEFT_REGION_HEIGHT = 0

    @JvmField var MIDDLE_REGION_X = 0
    @JvmField var MIDDLE_REGION_Y = 0
    @JvmField var MIDDLE_REGION_WIDTH = 0
    @JvmField var MIDDLE_REGION_HEIGHT = 0

    @JvmField var RIGHT_REGION_X = 0
    @JvmField var RIGHT_REGION_Y = 0
    @JvmField var RIGHT_REGION_WIDTH = 0
    @JvmField var RIGHT_REGION_HEIGHT = 0

    @JvmField var HEADING_P = 0.01
    @JvmField var HEADING_D = 0.0
    // TODO: WHY DID THIS WORK FINE AT 0.03 BEFORE
    @JvmField var LIFT_P = 0.6
    @JvmField var LIFT_I = 0.0
    @JvmField var LIFT_D = 0.0
    @JvmField var LIFT_F = 0.0
    // 16/24 ratio * 0.03937 * mm to inch * 111.715 mm spool circum / 103.8 ppr, in inches
    @JvmField var LIFT_DISTANCE_PER_PULSE = 0.6666*0.03937*111.715/103.8
    @JvmField var LIFT_MAX = 11.5
    @JvmField var LIFT_MIN = -0.2
    @JvmField var INTERMEDIARY_ZONE_1 = 0.01
    @JvmField var INTERMEDIARY_ZONE_2 = 10.6
    @JvmField var INTERMEDIARY_ZONE_3 = 2.0
    @JvmField var PIVOT_INTAKE = 0.76
    @JvmField var PIVOT_INTERMEDIARY = 0.72
    @JvmField var PIVOT_INTERMEDIARY_2 = 0.55
    @JvmField var PIVOT_OUTTAKE = 0.34
    @JvmField var PIVOT_PLOP = 0.34
    @JvmField var PIVOT_POOP = 0.28
    @JvmField var FINGER_L_OPEN = 0.45
    @JvmField var FINGER_L_CLOSE = 0.39
    @JvmField var FINGER_R_OPEN = 0.60
    @JvmField var FINGER_R_CLOSE = 0.68

    // outer roller auto stack angle for servo in degrees
    @JvmField var STACK_5 = 1
    @JvmField var STACK_4 = 0.9
    @JvmField var STACK_3 = 0.7
    @JvmField var STACK_2 = 0.6
    @JvmField var STACK_1 = 0.5

    @JvmField var AUTO = false
}
