package org.firstinspires.ftc.teamcode.util.gvf;

import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.path.Path;

public class GVF {
    // TODO: add ability for path sequences
    Path path;
    double kN;

    public GVF(Path path, double kN) {
        this.path = path;
        this.kN = kN;
    }

    public Vector2d getVector(Point p) {
        return path.getTau(p)
                .add(path.getError(p).scale(kN))
                .normalize();
    }
}
