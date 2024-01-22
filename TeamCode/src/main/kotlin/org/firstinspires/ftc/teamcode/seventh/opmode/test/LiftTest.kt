package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

@TeleOp(name = "LiftTest")
class LiftTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val gamepad = GamepadEx(gamepad1)
        val liftLeft = QuackMotor(hardwareMap, "liftLeft")
        val liftRight = QuackMotor(hardwareMap, "liftRight")
        val enc = QuackQuadrature(
            hardwareMap,
            "liftLeft",
            8192.0, 1.0/Globals.LIFT_DISTANCE_PER_PULSE,
            DcMotorSimple.Direction.FORWARD,
        )

            // TODO: is brake strong enough?
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD)
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE)
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        enc.reset()

        waitForStart();

        while (opModeIsActive() && !isStopRequested) {
            val power = gamepad.leftY/1.0
            liftLeft.setPower(power)
            liftRight.setPower(power)
            telemetry.addData("power", power);
            telemetry.addData("dist", enc.getDist());
            telemetry.addData("ticks", enc.getTicks());
            telemetry.addData("left current", liftLeft.getCurrent(CurrentUnit.AMPS))
            telemetry.addData("right current", liftRight.getCurrent(CurrentUnit.AMPS))
            telemetry.update();
        }
    }
}
