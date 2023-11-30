package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.drive.DriveConstants

import java.lang.Math.toDegrees

@TeleOp(name = "IMUTest")
class IMUTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        // dbval hardware = Hardware.getInstance(hardwareMap)
        // val drive = SampleMecanumDrive(hardware, hardwareMap)
        val dashboard = FtcDashboard.getInstance()
        val imu: IMU = hardwareMap.get(IMU::class.java, "imu")
        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)))
        var ang = 0.0
        var angVel = 0.0

        waitForStart();

        while (opModeIsActive() && !isStopRequested) {
            val packet = TelemetryPacket()
            ang = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            angVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
            packet.put("angle", toDegrees(ang))
            packet.put("velo", toDegrees(angVel))
            packet.put("ext first angle", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle)
            packet.put("ext second angle", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle)
            packet.put("ext three angle", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle)
            packet.put("int first angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle)
            packet.put("int second angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle)
            packet.put("int three angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle)
            dashboard.sendTelemetryPacket(packet)
            // drive.setWeightedDrivePower(Pose2d(0.0, 0.0, (-gamepad1.right_stick_x).toDouble()))
            // telemetry.addData("imu angle", toDegrees(hardware.imuAngle))
            // telemetry.addData("imu velocity", toDegrees(hardware.imuAngularVelo))
            // telemetry.update()
        }
    }
}
