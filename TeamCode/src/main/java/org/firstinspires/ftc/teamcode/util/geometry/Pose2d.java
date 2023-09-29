package org.firstinspires.ftc.teamcode.util.geometry;

public class Pose2d {
    Point position;
    Angle heading;

    public Pose2d() {
        this.position = new Point();
        this.heading = new Angle();
    }
    public Pose2d(Point position, Angle heading) {
        this.position = position;
        this.heading = heading;
    }

    public double getHeading() {
        return heading.getAng();
    }

    public Point getPosition() {
        return position;
    }
}
