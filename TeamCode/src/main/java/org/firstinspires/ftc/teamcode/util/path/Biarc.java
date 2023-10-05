package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public class Biarc implements PathSegment {
    // d1 and d2 are equal
    private Arc arc1;
    private Arc arc2;

    private Pose2d pose1;
    private Pose2d pose2;

    public Biarc(Pose2d pose1, Pose2d pose2) {
        this.pose1 = pose1;
        this.pose2 = pose2;
    }

    private Pose2d getJoin(Pose2d p1, Pose2d p2, double d) {
        Point q1 = p1.getPosition()
                     .move(p1.getHeading()
                             .scale(d));
        Point q2 = p2.getPosition()
            // this heading needs to be reversed
                     .move(p2.getHeading()
                             .scale(d));
        Point j = new Point((q1.x + q2.x) / 2, (q1.y + q2.y) / 2);
        Vector2d t_j = new Vector2d(q2.x - q1.x, q2.y - q1.y).normalize();
        return new Pose2d(j, t_j);
    }

    @Override
    public Vector2d getEndVector() {
        return pose2.getHeading();
    }

    @Override
    public Point getClosestPoint(Point p) {
        if (arc1.getClosestPoint(p).displacement(p) < arc2.getClosestPoint(p).displacement(p)) {
            return arc1.getClosestPoint(p);
        } else {
            return arc2.getClosestPoint(p);
        }
    }

    @Override
    public Pose2d getEndPose() {
        return pose2;
    }

    @Override
    public Vector2d getTau() {

    }
}
