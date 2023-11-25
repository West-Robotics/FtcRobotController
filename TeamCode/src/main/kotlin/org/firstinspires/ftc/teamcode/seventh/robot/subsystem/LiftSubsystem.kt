package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature

import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

class LiftSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class LiftState {
        DOWN,
        UP,
    } var state = LiftState.DOWN
        private set

    var distance = 0.0
        private set
    var power = 0.0
        private set
    var velocity = 0.0
        private set
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
        liftPid.setOutputRange(0.0, 0.7)
        liftPid.setTolerance(5.0)
        liftPid.reset()
        liftPid.enable()
        update(state, -1)
    }

    override fun read() {
        distance = enc.getDist()
        velocity = enc.getLinearVelocity()
        // help
        // voltage = hardware.voltage
    }

    fun update(s: LiftState, height: Int) {
        state = s
        liftPid.setpoint = when (s) {
            LiftState.UP    -> Globals.LIFT_HEIGHTS[height]
            LiftState.DOWN  -> Globals.LIFT_MIN
        }
        power = liftPid.performPID(distance)
    }

    override fun write() {
        // can this be cleaner?
        if (velocity == 0.0 && distance < 0.1) {
            if (distance != 0.0) {
                enc.reset()
            }
            if (power < 0.0) {
                power = 0.0
            }
        } else if (liftPid.onTarget()) {
            power = 0.0
        } else if (power < 0.0) {
            power /= if (distance >= 1.0) {
                3.0
            } else {
                3.0
            }
        }
        power = power*13.0/voltage
        liftLeft.setPower(power)
        liftRight.setPower(power)
    }
}
