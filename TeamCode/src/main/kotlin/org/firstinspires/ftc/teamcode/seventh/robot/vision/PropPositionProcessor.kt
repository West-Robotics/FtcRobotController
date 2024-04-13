// package org.firstinspires.ftc.teamcode.seventh.robot.vision
//
// import android.graphics.Canvas
// import android.graphics.Color
// import android.graphics.Paint
// import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
// import org.firstinspires.ftc.vision.VisionProcessor
// import org.opencv.core.Core
// import org.opencv.core.Mat
// import org.opencv.core.Point
// import org.opencv.core.Rect
// import org.opencv.imgproc.Imgproc
// import kotlin.math.roundToInt
//
// class PropPositionProcessor : VisionProcessor {
//     enum class PropPosition {
//         LEFT,
//         MIDDLE,
//         RIGHT
//     }
//     private val left_pointA = Point(Globals.LEFT_REGION_X, Globals.LEFT_REGION_Y)
//     private val left_pointB = Point(Globals.LEFT_REGION_X + Globals.LEFT_REGION_WIDTH,
//             Globals.LEFT_REGION_Y + Globals.LEFT_REGION_HEIGHT)
//     private val middle_pointA = Point(Globals.MIDDLE_REGION_X, Globals.MIDDLE_REGION_Y)
//     private val middle_pointB = Point(Globals.MIDDLE_REGION_X + Globals.MIDDLE_REGION_WIDTH,
//             Globals.MIDDLE_REGION_Y + Globals.MIDDLE_REGION_HEIGHT)
//     private val right_pointA = Point(Globals.RIGHT_REGION_X, Globals.RIGHT_REGION_Y)
//     private val right_pointB = Point(Globals.RIGHT_REGION_X + Globals.RIGHT_REGION_WIDTH,
//             Globals.RIGHT_REGION_Y + Globals.RIGHT_REGION_HEIGHT)
//
//     private val left_rect = Rect(left_pointA, left_pointB)
//     private val middle_rect = Rect(middle_pointA, middle_pointB)
//     private val right_rect = Rect(right_pointA, right_pointB)
//
//     @Volatile private var position = PropPosition.LEFT
//
//     private var hsv = Mat()
//     private var leftMat = Mat()
//     private var middleMat = Mat()
//     private var rightMat = Mat()
//
//     override fun init(width: Int, height: Int, calibration: CameraCalibration?) { }
//
//     override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
//         Imgproc.cvtColor(frame ?: throw NullPointerException("frame was null"), hsv, Imgproc.COLOR_BGR2HSV);
//         leftMat = hsv.submat(left_rect)
//         middleMat = hsv.submat(middle_rect)
//         rightMat = hsv.submat(right_rect)
//         val leftSum = Core.sumElems(leftMat)
//         val middleSum = Core.sumElems(middleMat)
//         val rightSum = Core.sumElems(rightMat)
//
//         // find which area has the most saturation: should be where big fat cube is
//         maxOf(leftSum.`val`[1], middleSum.`val`[1], rightSum.`val`[1]).let {
//             when (it) {
//                 leftSum.`val`[1] -> {
//                     position = PropPosition.LEFT
//                 }
//                 middleSum.`val`[1] -> {
//                     position = PropPosition.MIDDLE
//                 }
//                 rightSum.`val`[1] -> {
//                     position = PropPosition.RIGHT
//                 }
//             }
//         }
//         return position
//     }
//
//     override fun onDrawFrame(canvas: Canvas?, onscreenWidth: Int, onscreenHeight: Int, scaleBmpPxToCanvasPx: Float, scaleCanvasDensity: Float, userContext: Any?) {
//         val selectedPaint = Paint()
//         selectedPaint.color = Color.GREEN
//         selectedPaint.style = Paint.Style.STROKE
//         selectedPaint.strokeWidth = 8*scaleCanvasDensity
//
//         val unselectedPaint = Paint()
//         unselectedPaint.set(selectedPaint)
//         unselectedPaint.color = Color.RED
//
//         val drawLeft = makeGraphicsRect(left_rect, scaleBmpPxToCanvasPx)
//         val drawMiddle = makeGraphicsRect(middle_rect, scaleBmpPxToCanvasPx)
//         val drawRight = makeGraphicsRect(right_rect, scaleBmpPxToCanvasPx)
//
//         when (userContext) {
//             PropPosition.LEFT -> {
//                 canvas?.drawRect(drawLeft, selectedPaint)
//                 canvas?.drawRect(drawMiddle, unselectedPaint)
//                 canvas?.drawRect(drawRight, unselectedPaint)
//             }
//             PropPosition.MIDDLE -> {
//                 canvas?.drawRect(drawLeft, unselectedPaint)
//                 canvas?.drawRect(drawMiddle, selectedPaint)
//                 canvas?.drawRect(drawRight, unselectedPaint)
//             }
//             PropPosition.RIGHT -> {
//                 canvas?.drawRect(drawLeft, unselectedPaint)
//                 canvas?.drawRect(drawMiddle, unselectedPaint)
//                 canvas?.drawRect(drawRight, selectedPaint)
//             }
//         }
//     }
//
//     fun getPosition(): PropPosition = position
//
//     private fun makeGraphicsRect(rect: Rect, scaleBmpPxToCanvasPx: Float): android.graphics.Rect {
//         val left = (rect.x * scaleBmpPxToCanvasPx).roundToInt()
//         val top = (rect.y * scaleBmpPxToCanvasPx).roundToInt()
//         val right = left + (rect.width * scaleBmpPxToCanvasPx).roundToInt()
//         val bottom = top + (rect.height * scaleBmpPxToCanvasPx).roundToInt()
//         return android.graphics.Rect(left, top, right, bottom)
//     }
// }