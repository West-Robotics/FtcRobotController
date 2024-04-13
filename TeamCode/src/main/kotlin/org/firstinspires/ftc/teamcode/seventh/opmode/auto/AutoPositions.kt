package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.path
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Alliance
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Start
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Lane
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.YellowSide
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Stack
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Park
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline.PropPosition
import java.lang.Math.toRadians

class AutoPositions(
        val alliance: Alliance,
        val start: Start,
        val lane: Lane,
        val yellowSide: YellowSide,
        val stack: Stack,
        val park: Park,
        val prop: PropPosition,
) {
    // === BASE DEFINITIONS ===
    // position is middle of bot
    val botOffset = 9.5
    val sideMult = if (alliance == Alliance.RED) 1 else -1
    val initPose = when {
        alliance == Alliance.RED && start == Start.BACKDROP -> Pose2d(Vector2d(12.0+4.0, -72.0+botOffset), Rotation2d(toRadians(90.0)))// was 3.5 offset
        // alliance == Alliance.BLUE && start == Start.BACKDROP -> Pose2d(Vector2d(12.0+4.0, 72.0-botOffset), Rotation2d(toRadians(-90.0)))
        // alliance == Alliance.RED && start == Start.AUDIENCE -> Pose2d(Vector2d(-36.0-4.0, -72.0+botOffset), Rotation2d(toRadians(90.0)))
        // alliance == Alliance.BLUE && start == Start.AUDIENCE -> Pose2d(Vector2d(-36.0-4.0, 72.0-botOffset), Rotation2d(toRadians(-90.0)))
        else -> Pose2d()
    }
    val propPose = when {
        prop == PropPosition.LEFT && alliance == Alliance.RED && start == Start.BACKDROP -> Pose2d(9.0, -36.5, toRadians(0.0))
        prop == PropPosition.MIDDLE && alliance == Alliance.RED && start == Start.BACKDROP -> Pose2d(20.0, -34.0, toRadians(0.0))
        prop == PropPosition.RIGHT && alliance == Alliance.RED && start == Start.BACKDROP -> Pose2d(32.0, -34.0, toRadians(0.0))
        // prop == PropPosition.LEFT && alliance == Alliance.BLUE && start == Start.BACKDROP -> Pose2d(Vector2d(32.0, 34.0), Rotation2d(toRadians(0.0)))
        // prop == PropPosition.MIDDLE && alliance == Alliance.BLUE && start == Start.BACKDROP -> Pose2d(Vector2d(20.0, 32.0), Rotation2d(toRadians(0.0)))
        // prop == PropPosition.RIGHT && alliance == Alliance.BLUE && start == Start.BACKDROP -> Pose2d(Vector2d(10.0, 36.5), Rotation2d(toRadians(0.0)))
        // prop == PropPosition.LEFT && alliance == Alliance.RED && start == Start.AUDIENCE -> Pose2d(Vector2d(-55.0, -14.0), Rotation2d(toRadians(90.0)))
        // prop == PropPosition.MIDDLE && alliance == Alliance.RED && start == Start.AUDIENCE -> Pose2d(Vector2d(-48.0, -14.0), Rotation2d(toRadians(90.0)))
        // prop == PropPosition.RIGHT && alliance == Alliance.RED && start == Start.AUDIENCE -> Pose2d(Vector2d(-30.0, -36.0), Rotation2d(toRadians(180.0)))
        // prop == PropPosition.LEFT && alliance == Alliance.BLUE && start == Start.AUDIENCE -> Pose2d(Vector2d(30.0, 36.0), Rotation2d(toRadians(180.0)))
        // prop == PropPosition.MIDDLE && alliance == Alliance.BLUE && start == Start.AUDIENCE -> Pose2d(Vector2d(-48.0, -14.0), Rotation2d(toRadians(-90.0)))
        // prop == PropPosition.RIGHT && alliance == Alliance.BLUE && start == Start.AUDIENCE -> Pose2d(Vector2d(-55.0, -14.0), Rotation2d(toRadians(-90.0)))
        else -> Pose2d()
    }
    val yellowPose = when {
        alliance == Alliance.RED && prop == PropPosition.LEFT -> Pose2d(49.0, -30.0, toRadians(0.0))
        alliance == Alliance.RED && prop == PropPosition.MIDDLE -> Pose2d(49.0, -36.5, toRadians(0.0))
        alliance == Alliance.RED && prop == PropPosition.RIGHT -> Pose2d(49.0, -43.25, toRadians(0.0))
        // alliance == Alliance.BLUE && prop == PropPosition.LEFT -> Pose2d(Vector2d(49.0, 43.5), Rotation2d(toRadians(0.0)))
        // alliance == Alliance.BLUE && prop == PropPosition.MIDDLE -> Pose2d(Vector2d(49.0, 37.2), Rotation2d(toRadians(0.0)))
        // alliance == Alliance.BLUE && prop == PropPosition.RIGHT -> Pose2d(Vector2d(49.0, 32.5), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    } + if (yellowSide == YellowSide.RIGHT) Pose2d(Vector2d(0.0, -3.0), Rotation2d()) else Pose2d()
    // TODO: set up for mosaics or avoid the yellow
    val backdropPose = when {
        alliance == Alliance.RED && lane == Lane.LANE_1 -> Pose2d(49.0, -40.0+0.0, toRadians(0.0))
        alliance == Alliance.RED && lane == Lane.LANE_3 -> Pose2d(49.0, -40.0+0.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_1 -> Pose2d(49.0, 40.0-0.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_3 -> Pose2d(49.0, 40.0-0.0, toRadians(0.0))
        else -> Pose2d()
    }
    val stackPose = when {
        alliance == Alliance.RED && stack == Stack.FAR -> Pose2d(-72.0+12.0, -36.0, toRadians(0.0))
        alliance == Alliance.RED && stack == Stack.CLOSE -> Pose2d(-72.0+12.0, -36.0, toRadians(0.0))
        alliance == Alliance.BLUE && stack == Stack.FAR -> Pose2d(-72.0+12.0, 36.0, toRadians(0.0))
        alliance == Alliance.BLUE && stack == Stack.CLOSE -> Pose2d(-72.0+12.0, 36.0, toRadians(0.0))
        else -> Pose2d()
    }
    val stack2Pose = when {
        alliance == Alliance.RED -> Pose2d(-72.0+12.0, -36.0, toRadians(0.0))
        alliance == Alliance.BLUE -> Pose2d(-72.0+12.0, 36.0, toRadians(0.0))
        else -> Pose2d()
    }
    val parkPose = when {
        alliance == Alliance.RED && park == Park.INNER -> Pose2d(Vector2d(45.0, -10.0))
        alliance == Alliance.BLUE && park == Park.INNER -> Pose2d(Vector2d(45.0, 10.0))
        alliance == Alliance.RED && park == Park.OUTER -> Pose2d(Vector2d(45.0, -10.0))
        alliance == Alliance.BLUE && park == Park.OUTER -> Pose2d(Vector2d(45.0, 10.0))
        park == Park.NONE -> backdropPose
        else -> Pose2d()
    }
    val audienceJoin = when {
        alliance == Alliance.RED && lane == Lane.LANE_2 -> Pose2d(-12.0, -38.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_2 -> Pose2d(-12.0, 38.0, toRadians(0.0))
        else -> Pose2d()
    }
    val backdropJoin = when {
        alliance == Alliance.RED && lane == Lane.LANE_1 -> Pose2d(24.0, -60.0, toRadians(0.0))
        alliance == Alliance.RED && lane == Lane.LANE_2 -> Pose2d(24.0, -36.0, toRadians(0.0))
        alliance == Alliance.RED && lane == Lane.LANE_3 -> Pose2d(24.0, -12.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_1 -> Pose2d(24.0, 60.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_2 -> Pose2d(24.0, 36.0, toRadians(0.0))
        alliance == Alliance.BLUE && lane == Lane.LANE_3 -> Pose2d(24.0, 12.0, toRadians(0.0))
        else -> Pose2d()
    }

    // === PATHS ===
    val purple = path {
        if (prop == PropPosition.MIDDLE) {
            line {
                label("start to purple")
                start(initPose.position + Vector2d(7.0, 0.0))
                end(propPose.position)
                constraints {
                    decelDist(12.0)
                    heading(propPose.heading.theta)
                }
            }
        } else if ((prop == PropPosition.RIGHT && alliance == Alliance.RED) || (prop == PropPosition.LEFT && alliance == Alliance.BLUE)) {
            hermite {
                label("close to backdrop prop")
                start {
                    pos(initPose.position); ang(toRadians(30.0*sideMult)); v(30.0)
                }
                end {
                    pos(propPose.position); ang(toRadians(180.0)); v(10.0)
                }
                constraints {
                    decelDist(12.0)
                    heading(propPose.heading.theta)
                }
            }
        } else if (alliance == Alliance.RED) {
            hermite {
                label("red to left prop")
                start {
                    pos(initPose.position); ang(toRadians(20.0*(if ((alliance == Alliance.RED && start == Start.BACKDROP) || (alliance == Alliance.BLUE && start == Start.AUDIENCE))
                        1.0 else -1.0))); v(25.0)
                }
                end {
                    pos(propPose.position); ang(toRadians(180.0)); v(15.0)
                }
                constraints {
                    decelDist(12.0)
                    heading(propPose.heading.theta)
                }
            }
        } else {
            hermite {
                label("blue to right prop")
                start {
                    pos(initPose.position); ang(toRadians(-20.0)); v(25.0)
                }
                end {
                    pos(propPose.position); ang(toRadians(180.0)); v(15.0)
                }
                constraints {
                    decelDist(12.0)
                    heading(propPose.heading.theta)
                }
            }
        }
    }
    // val plusOne = path {
    //     line {
    //         label("purple to stack (far side)")
    //         start(purple.paths[0].endPose.position)
    //         end(stackPose.position + Vector2d(0.0, if (yellowSide == YellowAlliance.LEFT) 2.0 else -2.0))
    //         constraints {
    //             decelDist(16.0)
    //             heading(stackPose.heading.polarAngle)
    //         }
    //     }
    // }
    // val yellow = path {
    //     if (start == Start.CLOSE) {
    //         line {
    //             label("purple to yellow")
    //             start(propPose.position)
    //             end(yellowPose.position)
    //             constraints {
    //                 decelDist(16.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     } else {
    //         line {
    //             label("stack to backdrop join")
    //             start(stackPose.position)
    //             end(backdropJoin.position)
    //             constraints { heading(toRadians(0.0)) }
    //         }
    //         hermite {
    //             label("join to backdrop")
    //             start {
    //                 pos(backdropJoin.position)
    //                 ang(toRadians(0.0))
    //                 v(40.0)
    //             }
    //             end {
    //                 pos(yellowPose.position)
    //                 ang(toRadians(60.0))
    //                 v(40.0)
    //             }
    //             constraints {
    //                 decelDist(18.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     }
    // }
    // val score = path {
    //     if (lane == Lane.LANE_2) {
    //         line {
    //             label("stacks to backdrop")
    //             start(stackPose.position)
    //             end(backdropPose.position)
    //             constraints {
    //                 decelDist(18.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     } else if (lane == Lane.LANE_3) {
    //         line {
    //             label("stack to backdrop join")
    //             start(stackPose.position)
    //             end(backdropJoin.position)
    //             constraints { heading(toRadians(0.0)) }
    //         }
    //         hermite {
    //             label("join to backdrop")
    //             start {
    //                 pos(backdropJoin.position)
    //                 ang(toRadians(0.0))
    //                 v(30.0)
    //             }
    //             end {
    //                 pos(backdropPose.position)
    //                 ang(toRadians(70.0*-sideMult))
    //                 v(20.0)
    //             }
    //             constraints {
    //                 decelDist(8.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     }
    // }
    // val intake = path {
    //     if (lane == Lane.LANE_2) {
    //         line {
    //             label("backdrop to join")
    //             start(backdropPose.position)
    //             end(stackPose.position)
    //             constraints {
    //                 decelDist(32.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     } else if (lane == Lane.LANE_3) {
    //         hermite {
    //             label("backdrop to join")
    //             start {
    //                 pos(backdropPose.position)
    //                 ang(toRadians(120.0*sideMult))
    //                 v(40.0)
    //             }
    //             end {
    //                 pos(backdropJoin.position)
    //                 ang(toRadians(180.0))
    //                 v(40.0)
    //             }
    //             constraints { heading(toRadians(0.0)) }
    //         }
    //         line {
    //             label("backdrop join to stack")
    //             start(backdropJoin.position)
    //             end(stackPose.position)
    //             constraints {
    //                 decelDist(16.0)
    //                 heading(toRadians(0.0))
    //             }
    //         }
    //     }
    // }
    // val parkPath = path {
    //     line {
    //         label("backdrop to park")
    //         start(backdropPose.position)
    //         end(parkPose.position)
    //         constraints {
    //             decelDist(12.0)
    //             heading(toRadians(0.0))
    //         }
    //     }
    // }









// if (start == Start.FAR) {
//     hermite {
//         label("plus one stack to join")
//         start {
//             pos(stackPose.position)
//             ang(toRadians(0.0))
//             v(40.0)
//         }
//         end {
//             pos(audienceJoin.position)
//             ang(toRadians(0.0))
//             v(40.0)
//         }
//         constraints { heading(toRadians(0.0)) }
//     }
//     line {
//         label("audience join to backdrop join")
//         start(audienceJoin.position)
//         end(backdropJoin.position)
//         constraints { heading(toRadians(0.0)) }
//     }
//     hermite {
//         label("backdrop join to yellow")
//         start {
//             pos(backdropJoin.position)
//             ang(0.0)
//             v(40.0)
//         }
//         end {
//             pos(yellowPose.position)
//             ang(0.0)
//             v(40.0)
//         }
//         constraints {
//             decelDist(12.0)
//             heading(toRadians(0.0))
//         }
//     }
// } else {
    // hermite {
    //     label("stacks to join")
    //     start {
    //         pos(stackPose.position)
    //         ang(toRadians(0.0))
    //         v(40.0)
    //     }
    //     end {
    //         pos(audienceJoin.position)
    //         ang(toRadians(0.0))
    //         v(40.0)
    //     }
    //     constraints { heading(toRadians(0.0)) }
    // }
    // line {
    //     label("audience join to backdrop join")
    //     start(audienceJoin.position)
    //     end(backdropJoin.position)
    //     constraints { heading(toRadians(0.0)) }
    // }
    // hermite {
    //     label("backdrop join to backdrop")
    //     start {
    //         pos(backdropJoin.position)
    //         ang(0.0)
    //         v(40.0)
    //     }
    //     end {
    //         pos(backdropPose.position)
    //         ang(0.0)
    //         v(40.0)
    //     }
    //     constraints {
    //         decelDist(12.0)
    //         heading(toRadians(0.0))
    //     }
    // }
    // for far side white +1
}