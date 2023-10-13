//package org.firstinspires.ftc.teamcode.util.path;
//
//import org.firstinspires.ftc.teamcode.util.geometry.Angle;
//import org.firstinspires.ftc.teamcode.util.geometry.Point;
//import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
//import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
//
//import java.util.ArrayList;
//
//public class Path {
//    // TODO: guarantee c0, 1, 2 continuity
//    ArrayList<PathSegment> path;
//
//    public Path(PathBuilder builder) {
//        path = builder.path;
//    }
//
//    public Vector2d getTau(Point p) {
//        return path.get(getClosestIndex(p)).getTau();
//    }
//
//    public Vector2d getError(Point p) {
//        return path.get(getClosestIndex(p))
//                   .getClosestPoint(p)
//                   .toVector()
//                   .sub(p.toVector());
//    }
//
//    public int getClosestIndex(Point p) {
//        double closestDist = Double.MAX_VALUE;
//        int index = 0;
//        for (int i = 0; i < path.size()-1; i++) {
//            if (path.get(i).getClosestPoint(p).displacement(p) < closestDist) {
//                // maybe clean up double call here
//                closestDist = path.get(i).getClosestPoint(p).displacement(p);
//                index = i;
//            }
//        }
//        return index;
//    }
//
//    public Point getClosestPoint(Point p) {
//        return path.get(getClosestIndex(p)).getClosestPoint(p);
//    }
//
//    public static class PathBuilder {
//        ArrayList<PathSegment> path;
//        Pose2d initPose;
//        Pose2d lastPose;
//
//        public PathBuilder(Pose2d initPose) {
//            this.initPose = initPose;
//            lastPose = initPose;
//        }
//
//        public PathBuilder lineTo(Pose2d pose) {
//            path.add(new Line(lastPose.getPosition(), pose.getPosition()));
//            lastPose = pose;
//            return this;
//        }
//
//        public PathBuilder biarcTo(Pose2d pose) {
//            path.add(new Biarc(lastPose, pose));
//            lastPose = pose;
//            return this;
//        }
//
//        public Path build() {
//            return new Path(this);
//        }
//    }
//}
