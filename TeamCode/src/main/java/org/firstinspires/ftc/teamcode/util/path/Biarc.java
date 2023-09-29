package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public class Biarc extends PathSegment{
    private Arc arc1;
    private Arc arc2;

    private Pose2d pose1;
    private Pose2d pose2;

    public Biarc(Pose2d pose1, Pose2d pose2) {
        this.pose1 = pose1;
        this.pose2 = pose2;
    }
}
