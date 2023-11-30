package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot

class LiftSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var desiredHeight = 0
        private set
    var distance = 0.0
        private set
    var power = 0.0
        private set
    var velocity = 0.0
        private set
    var current = 0.0
        private set
    var grounded = true
    private val liftPid = PIDController(Globals.LIFT_P, Globals.LIFT_I, Globals.LIFT_D)
    var voltage = 13.0

    val liftLeft = QuackMotor(hardwareMap, "liftLeft")
    val liftRight = QuackMotor(hardwareMap, "liftRight")
    val enc = QuackQuadrature(hardwareMap, "liftLeft", 155.7, 1.0/Globals.LIFT_DISTANCE_PER_PULSE)
    val otherEnc = QuackQuadrature(hardwareMap, "liftRight", 155.7, 1.0/Globals.LIFT_DISTANCE_PER_PULSE)

    init {
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD)
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE)
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        enc.setDirection(DcMotorSimple.Direction.FORWARD)
        enc.reset()
        otherEnc.setDirection(DcMotorSimple.Direction.REVERSE)
        otherEnc.reset()
        liftPid.setOutputRange(0.0, 1.0)
        liftPid.setTolerance(5.0)
        liftPid.reset()
        liftPid.enable()
        update(-1)
    }

    override fun read() {
        distance = enc.getDist()
        velocity = enc.getLinearVelocity()
        current = liftLeft.getCurrent(CurrentUnit.AMPS) + liftRight.getCurrent(CurrentUnit.AMPS)
        voltage = Robot.voltage
    }

    fun update(height: Int) {
        desiredHeight = height
        if (height != 0) {
            grounded = false
        }
        liftPid.setpoint = when (desiredHeight) {
            -1 -> Globals.LIFT_HEIGHTS.last()
            else -> Globals.LIFT_HEIGHTS[desiredHeight]
        }
        power = liftPid.performPID(distance)
    }

    override fun write() {
        // can this be cleaner?
        if (current > 2.5 && distance < 0.75 && desiredHeight == 0) {
            if (distance != 0.0) {
                enc.reset()
                power = 0.0
                grounded = true
            }
        } else if (liftPid.onTarget()) {
            power = 0.0
        } else if (power < 0.0) {
            power /= if (distance >= 1.0) {
                2.5
            } else {
                2.5
            }
        }

        if (grounded) {
            power = 0.0
        }

        power = power*13.3/voltage
        liftLeft.setPower(power)
        liftRight.setPower(power)
    }
}
