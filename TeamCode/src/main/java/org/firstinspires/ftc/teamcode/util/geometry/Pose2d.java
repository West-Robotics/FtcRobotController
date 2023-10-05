package org.firstinspires.ftc.teamcode.util.geometry;

public class Pose2d {
    Point position;
    // TODO: make sure this is always normalized
    Vector2d heading;

    public Pose2d() {
        this.position = new Point();
        this.heading = new Vector2d();
    }
    public Pose2d(Point position, Vector2d heading) {
        this.position = position;
        this.heading = heading.normalize();
    }

    public Vector2d getHeading() { return heading; }

    public Angle toAng() { return heading.getAng(); }

    public Point getPosition() {
        return position;
    }
}
