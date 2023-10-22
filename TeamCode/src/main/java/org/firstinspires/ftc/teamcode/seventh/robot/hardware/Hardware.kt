package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

// directory structure and robot hardware thing heavily inspired by 16379 KookyBotz - they're pretty epic

class Hardware(val hardwareMap: HardwareMap) {
    // UH THIS DOESN'T WORK
    companion object {
        lateinit var instance: Hardware
            private set
        fun getInstance(hardwareMap: HardwareMap): Hardware {
            if (!this::instance.isInitialized) {
                instance = Hardware(hardwareMap)
            }
            return instance
        }
    }

    // TODO: uh what was my reasoning for not having dt first in the loop again
    // maybe it was only doing dt every other loop?
    // less important than slides? this season probably other way around
    @JvmField val leftFront: DcMotorEx
    @JvmField val leftRear: DcMotorEx
    @JvmField val rightRear: DcMotorEx
    @JvmField val rightFront: DcMotorEx

    val imu: IMU
    var imuAngle = 0.0
        private set
    var imuAngularVelo = 0.0
        private set
    private var imuOffset = 0.0

    // @JvmField val plPod: Encoder
    // @JvmField val ppPod: Encoder

    val liftLeft: DcMotorEx
    val liftRight: DcMotorEx
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

    // // TODO: read EasyOpenCV guide on vision portal
    // // randomization task, maybe detect waffles, also does AprilTags
    // public OpenCvCamera inCam
    // // auto-stop to not hit the board
    // public OpenCvCamera outCam
    // // one of these cams will be used for the randomization task
//  //   public PropDetection propDetection
    // public AprilTagProcessor aprilTag
    // // ALWAYS CLOSE AT THE END OF AUTO, maybe bake into command
    // public VisionPortal visionPortal
    // public GetPropPositionPipeline propPosition

    // TODO: replace with Photon voltage reader
    var voltage = 13.0
        private set
    private var voltageTimer: ElapsedTime

    init {
        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)))

        // power:
        //  ___________________
        // |   | chub   | ehub |
        // | 0 | intake | lf   |
        // | 1 | liftL  | lr   |
        // | 2 | liftR  | rr   |
        // | 3 | hang   | rf   |
        // --------------------

        // encoder:
        //  ___________________
        // |   | chub   | ehub |
        // | 0 | pl     |      |
        // | 1 | liftL  |      |
        // | 2 | liftR  |      |
        // | 3 | pp     |      |
        // --------------------

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
//        outerPivotLeft = hardwareMap.get(ServoImplEx::class.java, "outerPivotLeft")
//        outerPivotRight = hardwareMap.get(ServoImplEx::class.java, "outerPivotRight")

        hang = hardwareMap.get(DcMotorEx::class.java, "hang")

        voltageTimer = ElapsedTime()

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

    fun reset() {
        imuOffset = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) - imuOffset
        liftLeftEnc.reset()
        liftRightEnc.reset()
    }

}
