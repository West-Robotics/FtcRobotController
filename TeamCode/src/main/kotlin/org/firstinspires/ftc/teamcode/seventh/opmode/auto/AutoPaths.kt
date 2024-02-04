package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.path
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Side
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Start
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Lane
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.YellowSide
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Stack
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.Park
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline.PropPosition
import java.lang.Math.toRadians

class AutoPaths(
        val side: Side,
        val start: Start,
        val lane: Lane,
        val yellowSide: YellowSide,
        val stack: Stack,
        val park: Park,
        val prop: PropPosition,
) {
    // === BASE DEFINITIONS ===
    // position is middle of bot
    val botOffset = 10.875
    val initPose = when {
        side == Side.RED && start == Start.CLOSE -> Pose2d(Vector2d(12.0+3.5, -72.0+botOffset), Rotation2d(toRadians(90.0)))// was 3.5 offset
        side == Side.BLUE && start == Start.CLOSE -> Pose2d(Vector2d(12.0+3.5, 72.0-botOffset), Rotation2d(toRadians(-90.0)))
        else -> Pose2d()
    }
    val propPose = when {
                prop == PropPosition.LEFT && side == Side.RED && start == Start.CLOSE -> Pose2d(Vector2d(8.0, -36.0), Rotation2d(toRadians(0.0)))
                prop == PropPosition.MIDDLE && side == Side.RED && start == Start.CLOSE -> Pose2d(Vector2d(20.0, -33.0), Rotation2d(toRadians(0.0)))
                prop == PropPosition.RIGHT && side == Side.RED && start == Start.CLOSE -> Pose2d(Vector2d(30.0, -34.0), Rotation2d(toRadians(0.0)))
                prop == PropPosition.LEFT && side == Side.BLUE && start == Start.CLOSE -> Pose2d(Vector2d(30.0, 34.0), Rotation2d(toRadians(0.0)))
                prop == PropPosition.MIDDLE && side == Side.BLUE && start == Start.CLOSE -> Pose2d(Vector2d(20.0, 33.0), Rotation2d(toRadians(0.0)))
                prop == PropPosition.RIGHT && side == Side.BLUE && start == Start.CLOSE -> Pose2d(Vector2d(8.0, 36.0), Rotation2d(toRadians(0.0)))
                else -> Pose2d()
    }
    val stackPose = when {
        side == Side.RED && stack == Stack.CLOSE -> Pose2d(Vector2d(-72.0+botOffset+6.0, -38.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && stack == Stack.CLOSE -> Pose2d(Vector2d(-72.0+botOffset+6.0, 38.0), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    }
    val backdropPose = when {
        side == Side.RED -> Pose2d(Vector2d(48.0,-38.0+0.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE -> Pose2d(Vector2d(48.0,38.0-0.0), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    }
    val yellowPose = when {
        side == Side.RED && prop == PropPosition.LEFT -> Pose2d(Vector2d(47.0, -31.0), Rotation2d(toRadians(0.0)))
        side == Side.RED && prop == PropPosition.MIDDLE -> Pose2d(Vector2d(47.0, -37.0), Rotation2d(toRadians(0.0)))
        side == Side.RED && prop == PropPosition.RIGHT -> Pose2d(Vector2d(47.0, -41.5), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && prop == PropPosition.LEFT -> Pose2d(Vector2d(47.0, 41.5), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && prop == PropPosition.MIDDLE -> Pose2d(Vector2d(47.0, 37.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && prop == PropPosition.RIGHT -> Pose2d(Vector2d(47.0, 31.0), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    } + if (yellowSide == YellowSide.RIGHT) Pose2d(Vector2d(0.0, -3.0), Rotation2d()) else Pose2d()
    val parkPose = when {
        side == Side.RED && park == Park.INNER -> Pose2d(Vector2d(72.0-12.0-botOffset-5.0, -8.0))
        side == Side.BLUE && park == Park.INNER -> Pose2d(Vector2d(72.0-12.0-botOffset-5.0, 8.0))
        else -> Pose2d()
    }

    // === PATHS ===
    val outOfTheWay = path {
        line {
            label("start to out")
            start(initPose.position)
            end(initPose.position + Vector2d(7.0, 0.0))
            constraints {
                decelDist(10.0)
                heading(propPose.heading.polarAngle)
            }
        }
    }
    val purple = path {
        if (prop == PropPosition.MIDDLE || (prop == PropPosition.RIGHT && side == Side.RED) || (prop == PropPosition.LEFT && side == Side.BLUE)) {
            line {
                label("start to purple")
                start(initPose.position + Vector2d(7.0, 0.0))
                end(propPose.position)
                constraints {
                    decelDist(16.0)
                    heading(propPose.heading.polarAngle)
                }
            }
        } else if (side == Side.RED) {
            hermite {
                label("red to left prop")
                start {
                    pos(initPose.position); ang(toRadians(20.0)); v(25.0)
                }
                end {
                    pos(propPose.position); ang(toRadians(180.0)); v(15.0)
                }
                constraints {
                    decelDist(16.0)
                    heading(propPose.heading.polarAngle)
                }
            }
        } else {
            hermite {
                label("blue to right prop")
                start {
                    pos(initPose.position); ang(toRadians(-45.0)); v(15.0)
                }
                end {
                    pos(propPose.position); ang(toRadians(180.0)); v(15.0)
                }
                constraints {
                    decelDist(16.0)
                    heading(propPose.heading.polarAngle)
                }
            }
        }
    }
    val yellow = path {
        line {
            label("purple to yellow")
            start(propPose.position)
            end(yellowPose.position)
            constraints {
                decelDist(16.0)
                heading(toRadians(0.0))
            }
        }
    }
    val score = path {
        line {
            label("stacks to backdrop")
            start(stackPose.position)
            end(backdropPose.position)
            constraints {
                decelDist(16.0)
                heading(toRadians(0.0))
            }
        }
    }
    val intake = path {
        line {
            label("backdrop to join")
            start(backdropPose.position)
            end(stackPose.position)
            constraints {
                decelDist(24.0)
                heading(toRadians(0.0))
            }
        }
    }
    val intakeSlide = path {
        line {
            label("shift slide")
            start(stackPose.position)
            end(stackPose.position + if (side == Side.RED) Vector2d(0.0, -6.0) else Vector2d(0.0, +6.0))
            constraints {
                decelDist(10.0)
                heading(toRadians(0.0))
            }
        }
    }
    val parkPath = path {
        line {
            label("backdrop to park")
            start(backdropPose.position)
            end(parkPose.position)
            constraints {
                decelDist(16.0)
                heading(toRadians(0.0))
            }
        }
    }









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
    val audienceJoin = when {
        side == Side.RED && lane == Lane.LANE_2 -> Pose2d(Vector2d(-12.0, -38.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && lane == Lane.LANE_2 -> Pose2d(Vector2d(-12.0, 38.0), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    }
    val backdropJoin = when {
        side == Side.RED && lane == Lane.LANE_1 -> Pose2d(Vector2d(36.0, -60.0), Rotation2d(toRadians(0.0)))
        side == Side.RED && lane == Lane.LANE_2 -> Pose2d(Vector2d(36.0, -36.0), Rotation2d(toRadians(0.0)))
        side == Side.RED && lane == Lane.LANE_3 -> Pose2d(Vector2d(36.0, -12.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && lane == Lane.LANE_1 -> Pose2d(Vector2d(36.0, 60.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && lane == Lane.LANE_2 -> Pose2d(Vector2d(36.0, 36.0), Rotation2d(toRadians(0.0)))
        side == Side.BLUE && lane == Lane.LANE_3 -> Pose2d(Vector2d(36.0, 12.0), Rotation2d(toRadians(0.0)))
        else -> Pose2d()
    }
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
    // hermite {
    //     label("backdrop to join")
    //     start {
    //         pos(backdropPose.position)
    //         ang(toRadians(180.0))
    //         v(40.0)
    //     }
    //     end {
    //         pos(backdropJoin.position)
    //         ang(toRadians(180.0))
    //         v(40.0)
    //     }
    //     constraints { heading(toRadians(0.0)) }
    // }
    // line {
    //     label("backdrop join to audience join")
    //     start(backdropJoin.position)
    //     end(audienceJoin.position)
    //     constraints { heading(toRadians(0.0)) }
    // }
    // hermite {
    //     label("audience join to stack")
    //     start {
    //         pos(audienceJoin.position)
    //         ang(180.0)
    //         v(40.0)
    //     }
    //     end {
    //         pos(stackPose.position)
    //         ang(180.0)
    //         v(40.0)
    //     }
    //     constraints {
    //         decelDist(12.0)
    //         heading(toRadians(0.0))
    //     }
    // }
    // for far side white +1
    val plusOne = path {
        line {
            label("purple to stack (far side)")
            start(purple.paths[0].endPose.position)
            end(stackPose.position)
            constraints {
                decelDist(12.0)
                heading(stackPose.heading.polarAngle)
            }
        }
    }
}