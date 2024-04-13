package org.firstinspires.ftc.teamcode.seventh.robot.vision

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import java.lang.Math.toRadians


// TODO: conflicting camera rez and closed devices?
class Vision(val hardwareMap: HardwareMap) {
    // === constants ===
    private val atagRobotOffset = Vector2d(0.0, 0.0)
    private val redBackdropPos = Vector2d(61.25, -35.5)
    private val blueBackdropPos = Vector2d(61.25, 35.5)
    // === eocv ===
    private val propCamMonitorViewId: Int = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
    )
    lateinit private var propCam: OpenCvWebcam
    // lateinit private var propCam = OpenCvCameraFactory.getInstance().createWebcam(
    //         hardwareMap.get(WebcamName::class.java, "propCam"),
    //         propCamMonitorViewId
    // )
    private val propPosition = GetPropPositionPipeline()

    // === vision portal ===
    // thanks veer (16379)
    private lateinit var atagName: WebcamName
    private lateinit var atagProcessor: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    fun getPropPosition(): GetPropPositionPipeline.PropPosition {
        return propPosition.getPosition()
    }

    /**
     * @return global field coordinate
     */
    fun getPosition(heading: Rotation2d): Vector2d? =
        atagProcessor.detections?.let {
            // (if (Globals.side == Globals.Side.RED) redBackdropPos else blueBackdropPos) - (it.fold(Vector2d()) { mean, det ->
            (it.fold(Vector2d()) { mean, det ->
                mean +
                (Rotation2d(-toRadians(det.ftcPose.yaw)) *
                Vector2d(det.ftcPose.y, -det.ftcPose.x) + // swapped on purpose
                Vector2d(0.0, when (det.id) {
                    1, 4 -> -6.0
                    2, 5 -> 0.0
                    3, 6 -> 6.0
                    else -> 0.0
                }))
            } / it.size.toDouble()) + heading * atagRobotOffset
        }

    fun initProp() {
        propCam.openCameraDeviceAsync( object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                propCam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            override fun onError(errorCode: Int) {}
        })
    }
    fun enableProp() {
        propCam.setPipeline(propPosition)
    }
    fun closeProp() {
        propCam.closeCameraDevice()
    }

    fun initAtag() {
        atagProcessor = AprilTagProcessor.Builder()
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build()

        visionPortal = VisionPortal.Builder()
                .setCamera(hardwareMap.get<WebcamName>(WebcamName::class.java, "atagCam"))
                .setCameraResolution(Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(atagProcessor)
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .build()
    }

    /**
     * ONLY CALL ON REOPEN, NOT INIT
     */
    fun enableAtag() {
        visionPortal.setProcessorEnabled(atagProcessor, true)
    }
    fun disableAtag() {
        visionPortal.setProcessorEnabled(atagProcessor, false)
    }
    fun closeAtag() {
        visionPortal.close()
    }

    fun getFps() = visionPortal.fps
    fun getDets() = atagProcessor?.detections

    // TODO: get fps, stream format, toggle atag, optimize gain/exposure/focus
    // disable prop
}