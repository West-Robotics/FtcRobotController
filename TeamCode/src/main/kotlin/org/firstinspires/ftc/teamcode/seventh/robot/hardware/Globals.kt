package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.acmerobotics.dashboard.config.Config
import com.scrapmetal.quackerama.control.Pose2d

@Config
object Globals {
    @JvmField var pose: Pose2d = Pose2d()
    @JvmField var LEFT_REGION_X = 20.0
    @JvmField var LEFT_REGION_Y = 380.0
    @JvmField var LEFT_REGION_WIDTH = 200.0
    @JvmField var LEFT_REGION_HEIGHT = 100.0

    @JvmField var MIDDLE_REGION_X = 400.0
    @JvmField var MIDDLE_REGION_Y = 380.0
    @JvmField var MIDDLE_REGION_WIDTH = 450.0
    @JvmField var MIDDLE_REGION_HEIGHT = 100.0

    @JvmField var RIGHT_REGION_X = 1000.0
    @JvmField var RIGHT_REGION_Y = 380.0
    @JvmField var RIGHT_REGION_WIDTH = 200.0
    @JvmField var RIGHT_REGION_HEIGHT = 100.0

    // TODO: WHY DID THIS WORK FINE AT 0.03 BEFORE
    @JvmField var LIFT_P = 1.2
    @JvmField var LIFT_D = 0.0
    // 4.398 in spool circum / 8192 ppr
    @JvmField var LIFT_DISTANCE_PER_PULSE = 4.398 / 8192
    @JvmField var LIFT_MAX = 30.0
    var LIFT_MIN = -2.0
    // each height is spaced 2.6 inches apart
    @JvmField var LIFT_HEIGHTS = doubleArrayOf(-0.2) +
                                 DoubleArray(5) { 2.6*it + 14.1 }
    // 0.76 -> 0.72 -> 0.33
    // -0.04, -0.39
    @JvmField var FINGER_OPEN = 0.23
    @JvmField var FINGER_CLOSE = 0.05

    // outer roller auto stack angle for servo
    @JvmField var STACK_HEIGHT = doubleArrayOf(0.19, 0.22, 0.25, 0.285, 0.3)

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
    enum class YellowSide {
        LEFT,
        RIGHT,
    }
    enum class Stack {
        CLOSE,
        MIDDLE,
        FAR,
    }
    enum class Park {
        INNER,
        OUTER,
    }
}
