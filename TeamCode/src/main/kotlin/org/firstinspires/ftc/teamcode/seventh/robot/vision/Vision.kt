package org.firstinspires.ftc.teamcode.seventh.robot.vision

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class Vision(hardwareMap: HardwareMap) {
    // === eocv ===
    val cameraMonitorViewId: Int = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
    )
    val camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName::class.java, "camera"),
            cameraMonitorViewId
    )
    val propPosition = GetPropPositionPipeline()

    // === vision portal ===
    val atagProcessor = AprilTagProcessor.easyCreateWithDefaults()
    val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "camera"))
            .addProcessor(atagProcessor)
            .setCameraResolution(Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(false)
            .setAutoStopLiveView(true)
            .build()

    init {
        camera.openCameraDeviceAsync( object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            override fun onError(errorCode: Int) {}
        })
        visionPortal.setProcessorEnabled(atagProcessor, false)
    }

    fun getPropPosition(): GetPropPositionPipeline.PropPosition {
        return propPosition.getPosition()
    }

    fun getDistance(): Double {
        return atagProcessor.freshDetections?.fold(0.0)
            { mean, det -> mean + det.ftcPose.y/atagProcessor.freshDetections.size } ?: 0.0
    }

    fun enableProp() {
        camera.setPipeline(propPosition)
    }

    fun disableProp() {
        camera.closeCameraDevice()
    }

    fun enableAtag() {
        visionPortal.setProcessorEnabled(atagProcessor, true)
    }

    fun disableAtag() {
        visionPortal.setProcessorEnabled(atagProcessor, false)
    }

    // TODO: get fps, stream format, toggle atag, optimize gain/exposure/focus
    // disable prop
}