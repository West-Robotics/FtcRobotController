package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Angle;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

import java.util.Vector;

public class Path {
    Vector<PathSegment> path;

    public Path(PathBuilder builder) {
        path = builder.path;
    }

    public static class PathBuilder {
        Vector<PathSegment> path;
        Pose2d initPose;

        public PathBuilder(Pose2d initPose) {
            this.initPose = initPose;
        }

        public PathBuilder lineTo(Pose2d pose) {
            path.add(new Line(new Point(0, 0), new Point(0, 0)));
            return this;
        }

        public PathBuilder biarcTo(Pose2d pose) {
            path.add(new Arc(1, new Angle(0), new Angle(Math.PI)));
            return this;
        }

        public Path build() {
            return new Path(this);
        }
    }
}
