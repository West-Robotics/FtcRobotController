package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.seventh.drive.DriveConstants
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.PropPositionProcessor
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.Subsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

// directory structure and robot hardware thing heavily inspired by 16379 KookyBotz - they're pretty epic

// TODO: intake redesign, odo pods, passive hang, drone, end effector optimization
// TODO: belts, break beams, what else?
// TODO: auto: auto aim for pixels
class Hardware(val hardwareMap: HardwareMap) {
    // UH THIS DOESN'T WORK
    companion object {
        lateinit var instance: Hardware
            private set
        fun getInstance(hardwareMap: HardwareMap): Hardware {
            // if (!this::instance.isInitialized) {
            //     instance = Hardware(hardwareMap)
            // }
            // return instance
            return Hardware(hardwareMap)
        }
    }

    @JvmField var leftFront: DcMotorEx
    @JvmField var leftRear: DcMotorEx
    @JvmField var rightRear: DcMotorEx
    @JvmField var rightFront: DcMotorEx

    val imu: IMU = hardwareMap.get(IMU::class.java, "imu")
    var imuAngle = 0.0
        private set
    var imuAngularVelo = 0.0
        private set
    private var imuOffset = 0.0

    // @JvmField val plPod: Encoder
    // @JvmField val ppPod: Encoder

    @JvmField val liftLeft: DcMotorEx
    @JvmField val liftRight: DcMotorEx
    val liftLeftEnc: Motor.Encoder
    val liftRightEnc: Motor.Encoder
    val limit: TouchSensor
    val pivot: ServoImplEx
    val fingerLeft: ServoImplEx
    val fingerRight: ServoImplEx

    val intake: DcMotorEx
    // val outerPivotLeft: ServoImplEx
    // val outerPivotRight: ServoImplEx

    val hang: DcMotorEx

    // lateinit var visionPortal: VisionPortal
    // lateinit var propCam: OpenCvCamera
    // lateinit var propPosition: GetPropPositionPipeline
    // // TODO: read EasyOpenCV guide on vision portal
    // // randomization task, maybe detect waffles, also does AprilTags
    // // auto-stop to not hit the board
    // // one of these cams will be used for the randomization task
    // !lateinit var aprilTag: AprilTagProcessor
    // // ALWAYS CLOSE AT THE END OF AUTO, maybe bake into command
    // !lateinit var propProcessor: PropPositionProcessor
    // !lateinit var visionPortal: VisionPortal

    // TODO: replace with Photon voltage reader
    var voltage = 13.0
        private set
    private var voltageTimer: ElapsedTime

    init {
        // imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)))
        // imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)))


        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftRear = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        rightRear = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")

        // plPod = Encoder(hardwareMap.get(DcMotorEx::class.java, "plEnc"))
        // ppPod = Encoder(hardwareMap.get(DcMotorEx::class.java, "ppEnc"))

        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        liftLeftEnc = MotorEx(hardwareMap, "liftLeft").encoder
        liftRightEnc = MotorEx(hardwareMap, "liftRight").encoder
        limit = hardwareMap.get(TouchSensor::class.java, "limit")
        pivot = hardwareMap.get(ServoImplEx::class.java, "pivot")
        fingerLeft = hardwareMap.get(ServoImplEx::class.java, "fingerLeft")
        fingerRight = hardwareMap.get(ServoImplEx::class.java, "fingerRight")

        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
//      //   outerPivotLeft = hardwareMap.get(ServoImplEx::class.java, "outerPivotLeft")
//      //   outerPivotRight = hardwareMap.get(ServoImplEx::class.java, "outerPivotRight")

        hang = hardwareMap.get(DcMotorEx::class.java, "hang")

        voltageTimer = ElapsedTime()

        if (Globals.AUTO) {
            // propProcessor = PropPositionProcessor()
            // aprilTag = AprilTagProcessor.easyCreateWithDefaults()
            // visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "propCam"), propProcessor, aprilTag)
            // val cameraMonitorViewId: Int = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName);
            // propCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "propCam"), cameraMonitorViewId);
            // propPosition = GetPropPositionPipeline()
            // propCam.openCameraDeviceAsync(object: OpenCvCamera.AsyncCameraOpenListener {
            //     override fun onOpened() {
            //         propCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            //     }

            //     override fun onError(errorCode: Int) { }
            // });
            // propCam.setPipeline(propPosition);

        }
             // propProcessor = PropPositionProcessor()
             // visionPortal = VisionPortal.Builder()
             //         .setCamera(hardwareMap.get(WebcamName::class.java, "propCam"))
             //         .setCameraResolution(Size(320, 240))
             //         .enableLiveView(true)
             //         .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
             //         .setAutoStopLiveView(false)
             //         .addProcessor(propProcessor)
             //         .build()
             // visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "propCam"), propProcessor)
             // visionPortal.setProcessorEnabled(propProcessor, true)
        // if (Globals.AUTO) {
//      //       propDetection = new PropDetection()
        //     aprilTag = new AprilTagProcessor.Builder()
        //             .setDrawAxes(false)
        //             .setDrawCubeProjection(false)
        //             .setDrawTagOutline(true)
        //             .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //             .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //             .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //             // TODO: calibrate lens intrinsics
//      //               .setLensIntrinsics()
        //             .build()
        //     visionPortal = new VisionPortal.Builder()
        //             .setCamera(hardwareMap.get(WebcamName.class, "inCam"))
        //             .setCameraResolution(new Size(1280, 720))
        //             .enableLiveView(true)
        //             .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        //             .setAutoStopLiveView(false)
        //             .addProcessor(aprilTag)
        //             .build()
        //     visionPortal.setProcessorEnabled(aprilTag, false)
        //     propPosition = new GetPropPositionPipeline()
        //     inCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "inCam"))
        //     inCam.setPipeline(propPosition)
        //     inCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        //         @Override
        //         public void onOpened() {
        //             inCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT)
        //         }

        //         @Override
        //         public void onError(int errorCode) {

        //         }
        //     })
        // }
        Globals.PIVOT_OUTTAKE = 0.46
        voltage = hardwareMap.voltageSensor.iterator().next().voltage
    }

    fun read(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.read()
        }
        if (voltageTimer.seconds() > 5.0) {
            voltage = hardwareMap.voltageSensor.iterator().next().voltage
        }
    }

    fun write(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.write()
        }
    }

    // public void stopCameras() {
    //     visionPortal.setProcessorEnabled(aprilTag, false)
    //     visionPortal.close()
    // }

    fun updateIMU() {
        imuAngle = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) - imuOffset
        imuAngularVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
    }

    fun getAng(): Double {
        // return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        // wtf is this
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle.toDouble()
    }

    fun getAngV(): Double {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
    }

    fun reset() {
        imuOffset = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) - imuOffset
        // liftLeftEnc.reset()
        // liftRightEnc.reset()
    }

    // fun stop() {
    //     if (this::visionPortal.isInitialized) {
    //         visionPortal.close()
    //     }
    // }

}
