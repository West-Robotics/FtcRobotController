package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Angle;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

import java.util.ArrayList;

public class Path {
    ArrayList<PathSegment> path;

    public Path(PathBuilder builder) {
        path = builder.path;
    }

    public static class PathBuilder {
        ArrayList<PathSegment> path;
        Pose2d initPose;
        Pose2d lastPose;

        public PathBuilder(Pose2d initPose) {
            this.initPose = initPose;
            lastPose = initPose;
        }

        public PathBuilder lineTo(Pose2d pose) {
            path.add(new Line(lastPose.getPosition(), pose.getPosition()));
            lastPose = pose;
            return this;
        }

        public PathBuilder biarcTo(Pose2d pose) {
            path.add(new Biarc(lastPose, pose));
            lastPose = pose;
            return this;
        }

        public Path build() {
            return new Path(this);
        }
    }
}
