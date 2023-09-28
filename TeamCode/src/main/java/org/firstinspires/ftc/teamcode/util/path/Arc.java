package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Angle;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public class Arc extends PathSegment {
    Point center = new Point(0, 0);

    double r;
    Angle theta_i;
    Angle centralA;
    Angle theta_f;
    Point beginPoint;
    Point endPoint;

    // we need the central angle because it gives directional information
    public Arc(double r, Angle theta_i, Angle centralA) {
        this.r = r;
        this.theta_i = theta_i;
        this.centralA = centralA;
        theta_f = theta_i.add(centralA);
        beginPoint = new Point(r*Math.cos(theta_i.getAng()) + center.x,
                               r*Math.sin(theta_i.getAng()) + center.y);
        endPoint = new Point(r*Math.cos(theta_f.getAng()) + center.x,
                             r*Math.sin(theta_f.getAng()) + center.y);
    }

    @Override
    public Point getClosestPoint(Point p) {
        Point projection = projectToCircle(p);
        if (contains(projection.getOrigAng())) {
            return new Point(projection.x + center.x, projection.y + center.y);
        } else if (projection.displacement(beginPoint) < projection.displacement(endPoint)) {
            return beginPoint;
        // you should probably check here anyway even though it's a logical impossibility to not be this
        } else {
            return endPoint;
        }
    }

    private Point projectToCircle(Point p) {
        // vector from arc center to p
        Vector2d v = new Vector2d(center.x - p.x, center.y - p.y);
        // turn into unit vector
        v = v.scale(1/v.getMag());
        // project onto circle
        v = v.scale(r);
        return v.toPoint();
    }

    // TODO: make this not extremely bad
    public boolean contains(Angle a) {
        // if going counter-clockwise
        if (centralA.getAng() >= 0) {
            // if arc endpoint does not cross 2pi
            if (theta_f.getAng() <= 2 * Math.PI) {
                return (theta_i.getAng() <= a.getAng()) &&
                       (a.getAng() <= theta_f.getAng());
            // if arc endpoint does cross 2pi
            } else if (theta_f.getAng() > 2 * Math.PI) {
                return (theta_i.getAng() <= a.getAng()) ||
                       (a.getAng() <= Angle.wrap(theta_f.getAng()));
            }
        // if going clockwise
        } else if (centralA.getAng() < 0) {
            // if arc endpoint does not cross 0
            if (theta_f.getAng() >= 0) {
                return (theta_f.getAng() <= a.getAng()) &&
                       (a.getAng() <= theta_i.getAng());
            // if arc endpoint does cross 0
            } else if (theta_f.getAng() < 0) {
                return (Angle.wrap(theta_f.getAng()) <= a.getAng()) ||
                       (a.getAng() <= theta_i.getAng());
            }
        }
        // this should never occur
        return false;
    }
}
