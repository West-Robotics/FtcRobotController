package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Rotation2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d
import kotlin.math.cos
import kotlin.math.sin

// fun path(init: Path.() -> Unit): Path {
//     val path = Path()
//     path.init()
//     return path
// }


fun trajectory(init: TrajectoryBuilder.() -> Unit): Trajectory {
    val t = TrajectoryBuilder()
    t.init()
    return t.build()
}

class TrajectoryBuilder {
    val paths = mutableListOf<Path>()
    fun build(): Trajectory {
        return Trajectory(paths)
    }
    fun hermite(init: HermiteBuilder.() -> Unit) {
        val h = HermiteBuilder()
        h.init()
        paths.add(h.build())
    }
    fun line(init: LineBuilder.() -> Unit) {
        val l = LineBuilder()
        l.init()
        paths.add(l.build())
    }
    inner class HermiteBuilder {
        var label = ""
        var startPose = Pose2d()
        var v0 = 0.0
        var endPose = Pose2d()
        var v1 = 0.0

        fun label(label: String) { this.label = label }
        // TODO: this is just to enforce C1 continuity, you should
        //  always preserve C0 continuity anyway
        fun tangentToPrevious() {
            paths.last().let {
                startPose = it.endPose
                // TODO: set v0
            }
        }
        fun start(init: Start.() -> Unit) {
            val s = Start()
            s.init()
        }
        fun end(init: End.() -> Unit) {
            val e = End()
            e.init()
        }
        fun build() = CubicHermite(label, startPose, startPose.heading.toVector()*v0, endPose, endPose.heading.toVector()*v1)
        inner class Start {
            fun position(x: Double, y: Double) {
                startPose = Pose2d(Vector2d(x, y), startPose.heading)
            }
            fun heading(theta: Double) {
                startPose = Pose2d(startPose.position, Rotation2d(cos(theta), sin(theta)))
            }
            fun v(v: Double) {
                v0 = v
            }
        }
        inner class End {
            fun position(x: Double, y: Double) {
                endPose = Pose2d(Vector2d(x, y), endPose.heading)
            }
            fun heading(theta: Double) {
                endPose = Pose2d(endPose.position, Rotation2d(cos(theta), sin(theta)))
            }
            fun v(v: Double) {
                v1 = v
            }
        }
    }
    inner class LineBuilder {
        var label = ""
        var startPose = Pose2d()
        var endPose = Pose2d()

        fun label(label: String) { this.label = label }
        // TODO: this is just to enforce C1 continuity, you should
        //  always preserve C0 continuity anyway
        fun tangentToPrevious() {
            paths.last().let {
                startPose = it.endPose
                // TODO: set v0
            }
        }
        fun start(init: Start.() -> Unit) {
            val s = Start()
            s.init()
        }
        fun end(init: End.() -> Unit) {
            val e = End()
            e.init()
        }
        fun build() = Line(label, startPose,  endPose)
        inner class Start {
            fun position(x: Double, y: Double) {
                startPose = Pose2d(Vector2d(x, y), startPose.heading)
            }
            fun heading(theta: Double) {
                startPose = Pose2d(startPose.position, Rotation2d(cos(theta), sin(theta)))
            }
        }
        inner class End {
            fun position(x: Double, y: Double) {
                endPose = Pose2d(Vector2d(x, y), endPose.heading)
            }
            fun heading(theta: Double) {
                endPose = Pose2d(endPose.position, Rotation2d(cos(theta), sin(theta)))
            }
        }
    }
}

class Trajectory(val paths: MutableList<Path>) {
}
class Foo {
    val t: Trajectory = trajectory {
        hermite {
            label("test path part 1")
            start {
                position(0.0, 0.0)
                heading(0.0)
                v(10.0)
            }
            end {
                position(20.0, 20.0)
                heading(90.0)
                v(20.0)
            }
        }
        hermite {
            label("test path part 2")
            tangentToPrevious()
            end {
                position(-72.0, 0.0)
                heading(270.0)
                v(0.0)
            }
        }
        line {
            label("le epic line")
            start {
                position(-72.0, 0.0)
                heading(270.0)
            }
            end {
                position(0.0, 0.0)
                heading(0.0)
            }
        }
    }
}

// class Trajectory {
//     // TODO: guarantee c0, 1, 2 continuity
//     ArrayList<PathSegment> path;
//
//     public Path(PathBuilder builder) {
//         path = builder.path;
//     }
//
//     public Vector2d getTau(Point p) {
//         return path.get(getClosestIndex(p)).getTau();
//     }
//
//     public Vector2d getError(Point p) {
//         return path.get(getClosestIndex(p))
//                    .getClosestPoint(p)
//                    .toVector()
//                    .sub(p.toVector());
//     }
//
//     public int getClosestIndex(Point p) {
//         double closestDist = Double.MAX_VALUE;
//         int index = 0;
//         for (int i = 0; i < path.size()-1; i++) {
//             if (path.get(i).getClosestPoint(p).displacement(p) < closestDist) {
//                 // maybe clean up double call here
//                 closestDist = path.get(i).getClosestPoint(p).displacement(p);
//                 index = i;
//             }
//         }
//         return index;
//     }
//
//     public Point getClosestPoint(Point p) {
//         return path.get(getClosestIndex(p)).getClosestPoint(p);
//     }
//
//     public static class PathBuilder {
//         ArrayList<PathSegment> path;
//         Pose2d initPose;
//         Pose2d lastPose;
//
//         public PathBuilder(Pose2d initPose) {
//             this.initPose = initPose;
//             lastPose = initPose;
//         }
//
//         public PathBuilder lineTo(Pose2d pose) {
//             path.add(new Line(lastPose.getPosition(), pose.getPosition()));
//             lastPose = pose;
//             return this;
//         }
//
//         public PathBuilder biarcTo(Pose2d pose) {
//             path.add(new Biarc(lastPose, pose));
//             lastPose = pose;
//             return this;
//         }
//
//         public Path build() {
//             return new Path(this);
//         }
//     }
// }