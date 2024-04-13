package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.acmerobotics.dashboard.config.Config
import com.scrapmetal.quackerama.control.Pose2d

@Config
object Globals {
    var pose: Pose2d = Pose2d()

    var AUTO = false
    enum class Alliance {
        RED,
        BLUE,
    } var alliance = Alliance.RED
    enum class Start {
        BACKDROP,
        AUDIENCE,
    } var start = Start.BACKDROP
    enum class Lane {
        LANE_1,
        LANE_2,
        LANE_3,
    } var lane = Lane.LANE_1
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
        NONE,
        OUTER,
    }
}
