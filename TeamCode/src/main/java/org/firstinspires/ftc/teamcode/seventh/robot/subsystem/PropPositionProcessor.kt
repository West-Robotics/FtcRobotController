package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GetPropPositionPipeline extends OpenCvPipeline {
    public enum PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final Scalar GREEN = new Scalar(0, 255, 0);

    private Point left_pointA = new Point(Globals.LEFT_REGION_X,
                                          Globals.LEFT_REGION_Y);
    private Point left_pointB = new Point(Globals.LEFT_REGION_X + Globals.LEFT_REGION_WIDTH,
                                          Globals.LEFT_REGION_Y + Globals.LEFT_REGION_HEIGHT);
    private Point middle_pointA = new Point(Globals.MIDDLE_REGION_X,
                                            Globals.MIDDLE_REGION_Y);
    private Point middle_pointB = new Point(Globals.MIDDLE_REGION_X + Globals.MIDDLE_REGION_WIDTH,
                                            Globals.MIDDLE_REGION_Y + Globals.MIDDLE_REGION_HEIGHT);
    private Point right_pointA = new Point(Globals.RIGHT_REGION_X,
                                           Globals.RIGHT_REGION_Y);
    private Point right_pointB = new Point(Globals.RIGHT_REGION_X + Globals.RIGHT_REGION_WIDTH,
                                           Globals.RIGHT_REGION_Y + Globals.RIGHT_REGION_HEIGHT);

    private Rect left_rect = new Rect(left_pointA,
                                      left_pointB);
    private Rect middle_rect = new Rect(middle_pointA,
                                        middle_pointB);
    private Rect right_rect = new Rect(right_pointA,
                                       right_pointB);

    private volatile PropPosition position = PropPosition.LEFT;

    private Mat hsv = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat leftMat = hsv.submat(left_rect);
        Mat middleMat = hsv.submat(middle_rect);
        Mat rightMat = hsv.submat(right_rect);
        Scalar leftSum = Core.sumElems(leftMat);
        Scalar middleSum = Core.sumElems(middleMat);
        Scalar rightSum = Core.sumElems(rightMat);

        // find which area has the most saturation: should be where big fat cube is
        double maxSaturation = Math.max(leftSum.val[1], Math.max(middleSum.val[1], rightSum.val[1]));

        if (maxSaturation == leftSum.val[1]) {
            position = PropPosition.LEFT;
            Imgproc.rectangle(input, left_pointA, left_pointB, GREEN);
        } else if (maxSaturation == middleSum.val[1]) {
            position = PropPosition.MIDDLE;
            Imgproc.rectangle(input, middle_pointA, middle_pointB, GREEN);
        } else if (maxSaturation == rightSum.val[1]) {
            position = PropPosition.RIGHT;
            Imgproc.rectangle(input, right_pointA, right_pointB, GREEN);
        }
        leftMat.release();
        middleMat.release();
        rightMat.release();
        return input;
    }

    public PropPosition getPosition() {
        return position;
    }
}
