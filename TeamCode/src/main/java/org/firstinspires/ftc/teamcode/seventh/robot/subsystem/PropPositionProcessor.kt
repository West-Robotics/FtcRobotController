package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class PropPositionProcessor : VisionProcessor {
    enum class PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private val GREEN = Scalar(0.0, 255.0, 0.0)
    private val left_pointA = Point(Globals.LEFT_REGION_X, Globals.LEFT_REGION_Y)
    private val left_pointB = Point(Globals.LEFT_REGION_X + Globals.LEFT_REGION_WIDTH,
                                    Globals.LEFT_REGION_Y + Globals.LEFT_REGION_HEIGHT)
    private val middle_pointA = Point(Globals.MIDDLE_REGION_X, Globals.MIDDLE_REGION_Y)
    private val middle_pointB = Point(Globals.MIDDLE_REGION_X + Globals.MIDDLE_REGION_WIDTH,
                                      Globals.MIDDLE_REGION_Y + Globals.MIDDLE_REGION_HEIGHT)
    private val right_pointA = Point(Globals.RIGHT_REGION_X, Globals.RIGHT_REGION_Y)
    private val right_pointB = Point(Globals.RIGHT_REGION_X + Globals.RIGHT_REGION_WIDTH,
                                     Globals.RIGHT_REGION_Y + Globals.RIGHT_REGION_HEIGHT)

    private val left_rect = Rect(left_pointA, left_pointB)
    private val middle_rect = Rect(middle_pointA, middle_pointB)
    private val right_rect = Rect(right_pointA, right_pointB)

    @Volatile private var position = PropPosition.LEFT

    private var hsv = Mat()
    private var leftMat = Mat()
    private var middleMat = Mat()
    private var rightMat = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) { }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
        leftMat = hsv.submat(left_rect)
        middleMat = hsv.submat(middle_rect)
        rightMat = hsv.submat(right_rect)
        val leftSum = Core.sumElems(leftMat)
        val middleSum = Core.sumElems(middleMat)
        val rightSum = Core.sumElems(rightMat)

        // find which area has the most saturation: should be where big fat cube is
        maxOf(leftSum.`val`[1], middleSum.`val`[1], rightSum.`val`[1]).let {
            when (it) {
                leftSum.`val`[1] -> {
                    position = PropPosition.LEFT
                    Imgproc.rectangle(frame, left_pointA, left_pointB, GREEN)
                }
                middleSum.`val`[1] -> {
                    position = PropPosition.MIDDLE
                    Imgproc.rectangle(frame, middle_pointA, left_pointB, GREEN)
                }
                rightSum.`val`[1] -> {
                    position = PropPosition.RIGHT
                    Imgproc.rectangle(frame, right_pointA, left_pointB, GREEN)
                }
            }
        }

        hsv.release()
        leftMat.release()
        middleMat.release()
        rightMat.release()
        // tf is this
        return frame!!
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) { }
    fun getPosition(): PropPosition {
        return position
    }
}