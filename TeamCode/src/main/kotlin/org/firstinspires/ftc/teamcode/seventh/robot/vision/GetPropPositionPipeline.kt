package org.firstinspires.ftc.teamcode.seventh.robot.vision

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class GetPropPositionPipeline : OpenCvPipeline() {
    private val LEFT_REGION_X = 20.0
    private val LEFT_REGION_Y = 400.0
    private val LEFT_REGION_WIDTH = 200.0
    private val LEFT_REGION_HEIGHT = 100.0

    private val MIDDLE_REGION_X = 400.0
    private val MIDDLE_REGION_Y = 400.0
    private val MIDDLE_REGION_WIDTH = 450.0
    private val MIDDLE_REGION_HEIGHT = 100.0

    private val RIGHT_REGION_X = 1000.0
    private val RIGHT_REGION_Y = 400.0
    private val RIGHT_REGION_WIDTH = 200.0
    private val RIGHT_REGION_HEIGHT = 100.0
    enum class PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private val GREEN = Scalar(0.0, 255.0, 0.0)
    private val RED = Scalar(255.0, 0.0, 0.0)
    private val left_pointA = Point(LEFT_REGION_X, LEFT_REGION_Y)
    private val left_pointB = Point(LEFT_REGION_X + LEFT_REGION_WIDTH,
        LEFT_REGION_Y + LEFT_REGION_HEIGHT)
    private val middle_pointA = Point(MIDDLE_REGION_X, MIDDLE_REGION_Y)
    private val middle_pointB = Point(MIDDLE_REGION_X + MIDDLE_REGION_WIDTH,
        MIDDLE_REGION_Y + MIDDLE_REGION_HEIGHT)
    private val right_pointA = Point(RIGHT_REGION_X, RIGHT_REGION_Y)
    private val right_pointB = Point(RIGHT_REGION_X + RIGHT_REGION_WIDTH,
        RIGHT_REGION_Y + RIGHT_REGION_HEIGHT)

    private val left_rect = Rect(left_pointA, left_pointB)
    private val middle_rect = Rect(middle_pointA, middle_pointB)
    private val right_rect = Rect(right_pointA, right_pointB)

    @Volatile private var position = PropPosition.LEFT

    private var hsv = Mat()
    private var leftMat = Mat()
    private var middleMat = Mat()
    private var rightMat = Mat()

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        leftMat = hsv.submat(left_rect)
        middleMat = hsv.submat(middle_rect)
        rightMat = hsv.submat(right_rect)
        val leftSum = Core.sumElems(leftMat)
        val middleSum = Core.sumElems(middleMat)
        val rightSum = Core.sumElems(rightMat)

        // find which area has the most saturation: should be where big fat cube is
        maxOf(leftSum.`val`[1], middleSum.`val`[1], rightSum.`val`[1]).let {
            if ((Globals.alliance == Globals.Alliance.RED && Globals.start == Globals.Start.BACKDROP) || (Globals.alliance == Globals.Alliance.BLUE && Globals.start == Globals.Start.AUDIENCE)) {
                when {
                    it == middleSum.`val`[1] && it > 1200000.0 -> {
                        position = PropPosition.MIDDLE
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, GREEN, 8)
                        Imgproc.rectangle(input, left_pointA, left_pointB, RED, 8)
                        Imgproc.rectangle(input, right_pointA, right_pointB, RED, 8)
                        position = PropPosition.MIDDLE
                    }
                    it == rightSum.`val`[1] && it > 1200000.0 -> {
                        position = PropPosition.RIGHT
                        Imgproc.rectangle(input, right_pointA, right_pointB, GREEN, 8)
                        Imgproc.rectangle(input, left_pointA, left_pointB, RED, 8)
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, RED, 8)
                        position = PropPosition.RIGHT
                    }
                    else -> {
                        position = PropPosition.LEFT
                        Imgproc.rectangle(input, left_pointA, left_pointB, GREEN, 8)
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, RED, 8)
                        Imgproc.rectangle(input, right_pointA, right_pointB, RED, 8)
                        position = PropPosition.LEFT
                    }
                }
            } else if ((Globals.alliance == Globals.Alliance.BLUE && Globals.start == Globals.Start.BACKDROP) || (Globals.alliance == Globals.Alliance.RED && Globals.start == Globals.Start.AUDIENCE)) {
                when {
                    it == middleSum.`val`[1] && it > 1200000.0 -> {
                        position = PropPosition.MIDDLE
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, GREEN, 8)
                        Imgproc.rectangle(input, left_pointA, left_pointB, RED, 8)
                        Imgproc.rectangle(input, right_pointA, right_pointB, RED, 8)
                        position = PropPosition.MIDDLE
                    }
                    it == leftSum.`val`[1] && it > 1200000.0 -> {
                        position = PropPosition.LEFT
                        Imgproc.rectangle(input, left_pointA, left_pointB, GREEN, 8)
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, RED, 8)
                        Imgproc.rectangle(input, right_pointA, right_pointB, RED, 8)
                        position = PropPosition.LEFT
                    }
                    else -> {
                        position = PropPosition.RIGHT
                        Imgproc.rectangle(input, right_pointA, right_pointB, GREEN, 8)
                        Imgproc.rectangle(input, left_pointA, left_pointB, RED, 8)
                        Imgproc.rectangle(input, middle_pointA, middle_pointB, RED, 8)
                        position = PropPosition.RIGHT
                    }
                }
            }
            // println("max sat $it")
        }

        hsv.release()
        leftMat.release()
        middleMat.release()
        rightMat.release()
        // tf is this
        // return frame!!
        return input
    }

    fun getPosition(): PropPosition {
        return position
    }

}






