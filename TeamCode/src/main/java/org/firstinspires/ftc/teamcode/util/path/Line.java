package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public class Line extends PathSegment {
    Point a;
    Point b;

    public Line(Point a, Point b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public Vector2d getEndVector() {
        return getTau();
    }

    @Override
    public Point getClosestPoint(Point p) {
        
    }

    @Override
    public Pose2d getEndPose() {
        return new Pose2d(b, getEndVector());
    }

    @Override
    public Vector2d getTau() {
        return new Vector2d(b.x - a.x, b.y - a.y).normalize();
    }
}
