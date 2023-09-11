package org.firstinspires.ftc.teamcode.seventh.robot.hardware;

import android.util.Size;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// directory structure and robot hardware thing heavily inspired by 16379 KookyBotz - they're pretty epic

public class Hardware {
    private static Hardware instance = null;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0.0;
    private double imuAngularVelo = 0.0;
    private double imuOffset = 0.0;

    public Encoder plPod;
    public Encoder ppPod;

    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public Motor.Encoder liftLeftEnc;
    public Motor.Encoder liftRightEnc;

    public DcMotorEx flip;

    public DcMotorEx hang;

    // randomization task, maybe detect waffles, also does AprilTags
    public OpenCvCamera inCam;
    // auto-stop to not hit the board
    public OpenCvCamera outCam;
    // one of these cams will be used for the randomization task
//    public PropDetection propDetection;
    public AprilTagProcessor aprilTag;
    // ALWAYS CLOSE AT THE END OF AUTO, maybe bake into command
    public VisionPortal visionPortal;
    public GetPropPositionPipeline propPosition;

    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    private HardwareMap hardwareMap;

    public static Hardware getInstance() {
        if (instance == null) {
            instance = new Hardware();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        synchronized (imuLock) {
            imu = hardwareMap.get(IMU.class, "imu");
        }

        // power:
        //  ___________________
        // |   | chub  | ehub |
        // | 0 | flip  | lf   |
        // | 1 | liftL | lr   |
        // | 2 | liftR | rr   |
        // | 3 | hang  | rf   |
        // --------------------

        // encoder:
        //  ___________________
        // |   | chub  | ehub |
        // | 0 | pl    |      |
        // | 1 | liftL |      |
        // | 2 | liftR/flip |      |
        // | 3 | pp    |      |
        // --------------------

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        plPod = new Encoder(hardwareMap.get(DcMotorEx.class, "plEnc"));
        ppPod = new Encoder(hardwareMap.get(DcMotorEx.class, "ppEnc"));

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeftEnc = new MotorEx(hardwareMap, "liftLeft").encoder;
        liftRightEnc = new MotorEx(hardwareMap, "liftRight").encoder;

        flip = hardwareMap.get(DcMotorEx.class, "flip");

        hang = hardwareMap.get(DcMotorEx.class, "hang");

        voltageTimer = new ElapsedTime();

        if (Globals.AUTO) {
//            propDetection = new PropDetection();
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    // TODO: calibrate lens intrinsics
//                    .setLensIntrinsics()
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "inCam"))
                    .setCameraResolution(new Size(1280, 720))
                    .enableCameraMonitoring(true)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setAutoStopLiveView(false)
                    .addProcessor(aprilTag)
                    .build();
            visionPortal.setProcessorEnabled(aprilTag, false);
            propPosition = new GetPropPositionPipeline();
            inCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "inCam"));
            inCam.setPipeline(propPosition);
            inCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    inCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void stopCameras() {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.close();
    }

    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public double getAngularVelocity() {
        return imuAngularVelo;
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                    imuAngularVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                    // uhhhh which one do i use
                    // imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
            }
        });
        imuThread.start();
    }

    public void reset() {
        imuOffset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
        liftLeftEnc.reset();
        liftRightEnc.reset();
    }

    public void getVoltage() {
        return voltage;
    }
}
