package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public interface PathSegment {
    // always return a normalized vector
    // i don't think this is actually necessary given we have getEndPose()
    public Vector2d getEndVector();
    public Point getClosestPoint(Point p);
    public Pose2d getEndPose();
    public Vector2d getTau();
}
