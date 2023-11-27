package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.acmerobotics.dashboard.config.Config

@Config
object Globals {
    @JvmField var LEFT_REGION_X = 20.0
    @JvmField var LEFT_REGION_Y = 420.0
    @JvmField var LEFT_REGION_WIDTH = 200.0
    @JvmField var LEFT_REGION_HEIGHT = 100.0

    @JvmField var MIDDLE_REGION_X = 600.0
    @JvmField var MIDDLE_REGION_Y = 420.0
    @JvmField var MIDDLE_REGION_WIDTH = 200.0
    @JvmField var MIDDLE_REGION_HEIGHT = 100.0

    @JvmField var RIGHT_REGION_X = 1080.0
    @JvmField var RIGHT_REGION_Y = 420.0
    @JvmField var RIGHT_REGION_WIDTH = 200.0
    @JvmField var RIGHT_REGION_HEIGHT = 100.0

    @JvmField var HEADING_P = 0.01
    @JvmField var HEADING_D = 0.0
    // TODO: WHY DID THIS WORK FINE AT 0.03 BEFORE
    @JvmField var LIFT_P = 0.6
    @JvmField var LIFT_I = 0.0
    @JvmField var LIFT_D = 0.0
    @JvmField var LIFT_F = 0.0
    // 4.398 in spool circum * 20/30 ratio / 103.8 ppr, in inches
    @JvmField var LIFT_DISTANCE_PER_PULSE = 4.398 * 0.6666 / 103.8
    @JvmField var LIFT_MAX = 30.0
    @JvmField var LIFT_MIN = -2.0
    // each height is spaced 4 inches apart
    @JvmField var LIFT_HEIGHTS = DoubleArray(5) { 3.0*it + 13.5 }
    @JvmField var INTERMEDIARY_ZONE_1 = 0.01
    @JvmField var INTERMEDIARY_ZONE_2 = 10.8
    @JvmField var INTERMEDIARY_ZONE_3 = 2.0
    // 0.76 -> 0.72 -> 0.33
    // -0.04, -0.39
    @JvmField var FINGER_L_OPEN = 0.05
    @JvmField var FINGER_L_CLOSE = 0.0
    @JvmField var FINGER_R_OPEN = 0.05
    @JvmField var FINGER_R_CLOSE = 0.0

    // outer roller auto stack angle for servo in degrees
    @JvmField var STACK_HEIGHT = doubleArrayOf(0.23, 0.25, 0.3, 0.35, 0.4)

    @JvmField var AUTO = false

    enum class Side {
        RED,
        BLUE,
    } @JvmField var side = Side.RED
    enum class Start {
        CLOSE,
        FAR,
    } @JvmField var start = Start.CLOSE
    enum class Lane {
        LANE_1,
        LANE_2,
        LANE_3,
    } @JvmField var lane = Lane.LANE_1
}
