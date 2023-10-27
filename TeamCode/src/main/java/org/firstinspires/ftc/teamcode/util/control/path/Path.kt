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


fun path(init: PathBuilder.() -> Unit): Path {
    val p = PathBuilder()
    p.init()
    return p.build()
}

// TODO: somehow add tangency preservation from spline -> line
// TODO: figure out end point behavior, rn the idea of converging to
//   the end point first and then proceeding to the next segment
//   acts as an auto-important-point-marker but is also kind of not clean and smooth
class PathBuilder {
    val paths = mutableListOf<PathSegment>()
    fun build(): Path {
        return Path(paths)
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
        fun build() = CubicHermite(label, startPose, startPose.heading.vector*v0, endPose, endPose.heading.vector*v1)
        inner class Start {
            fun pos(x: Double, y: Double) {
                startPose = Pose2d(Vector2d(x, y), startPose.heading)
            }
            fun ang(theta: Double) {
                startPose = Pose2d(startPose.position, Rotation2d(cos(theta), sin(theta)))
            }
            // v is for hermite
            fun v(v: Double) {
                v0 = v
            }
        }
        inner class End {
            fun pos(x: Double, y: Double) {
                endPose = Pose2d(Vector2d(x, y), endPose.heading)
            }
            fun ang(theta: Double) {
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
            fun pos(x: Double, y: Double) {
                startPose = Pose2d(Vector2d(x, y), startPose.heading)
            }
            fun ang(theta: Double) {
                startPose = Pose2d(startPose.position, Rotation2d(cos(theta), sin(theta)))
            }
        }
        inner class End {
            fun pos(x: Double, y: Double) {
                endPose = Pose2d(Vector2d(x, y), endPose.heading)
            }
            fun ang(theta: Double) {
                endPose = Pose2d(endPose.position, Rotation2d(cos(theta), sin(theta)))
            }
        }
    }
}

// to be clear, this only specifies the path that the robot will take.
// nothing about heading or velocity etc.
class Path(val paths: MutableList<PathSegment>) {
    // TODO: guarantee C0, optionally C1 continuity?
    //   compile time error for C0 discontinuities
    fun tauOf(p: Vector2d) = paths.minBy { it.eOf(p).mag }.tauOf(p)
    fun eOf(p: Vector2d) = paths.minOfWith(Vector2d.comparator) { it.eOf(p) }
}
class Foo {
    val p: Path = path {
        hermite {
            label("test path part 1")
            start { pos(  0.0,  0.0);   ang(  0.0); v(10.0) }
            end {   pos( 20.0, 20.0);   ang( 90.0); v(20.0) }
        }
        hermite {
            label("test path part 2")
            tangentToPrevious()
            end {   pos(-72.0,  0.0);   ang(270.0); v( 0.0) }
        }
        line {
            label("le epic line")
            start { pos(-72.0,  0.0);   ang(270.0) }
            end {   pos(  0.0,  0.0);   ang(  0.0) }
        }
    }
}

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