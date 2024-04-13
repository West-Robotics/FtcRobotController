package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.controller.PIDF
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.*

import kotlin.math.absoluteValue
import kotlin.math.floor

class LiftSubsystem(hardwareMap: HardwareMap) : Subsystem {
    // 0 is inside, 1 is first backdrop level
    var boardLevel: Double = 0.0
    // inches
    var extension: Double = 0.0
    var overCurrent: Boolean = false
    var grounded: Boolean = true
    var power: Double = 0.0
    private val pidf = PIDF(2.1, 0.0, 0.025)
    // 8192 / 4.398 in spool circum
    private val TICKS_PER_DISTANCE = 8192/4.398
    private var voltage = 13.3

    private val liftL = QuackMotor(hardwareMap, "liftL")
    private val liftR = QuackMotor(hardwareMap, "liftR")
    private val enc = QuackQuadrature(
        hardwareMap,
        "liftL",
        8192.0, TICKS_PER_DISTANCE,
        DcMotorSimple.Direction.FORWARD,
    )

    init {
        // TODO: is brake strong enough?
        liftL.setDirection(DcMotorSimple.Direction.REVERSE)
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        liftL.setCurrentAlert(2.50)
        liftR.setDirection(DcMotorSimple.Direction.FORWARD)
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        enc.reset()
        // will this cause to slam down in init due to not looping the current check?
        // write()
    }

    override fun read() {
        extension = enc.getDist()
        overCurrent = liftL.isOverCurrent()
        // voltage = Robot.voltage
    }

    fun set(l: Double) {
        boardLevel = l.coerceIn(0.0..8.0)
        if (boardLevel != 0.0) grounded = false
    }
    fun set(r: Roll) {
        boardLevel = floor(boardLevel) + when (r) {
            Roll.HORIZ -> 0.0
            Roll.LM, Roll.RM, Roll.LM_I, Roll.RM_I -> 0.5
            Roll.VERT, Roll.VERT_I -> 0.6
        }
        if (boardLevel != 0.0) grounded = false
    }
    fun set(l: Double, r: Roll) {
        set(l)
        set(r)
    }

    fun raise() = set(boardLevel+1)
    fun lower() = set(boardLevel-1)

    override fun write() {
        power = pidf.update(extension, levelToExtension(boardLevel), Robot.getDt())
        // reground if there's a current spike, we are low, want to be low, and are not reset
        if (overCurrent && extension < 0.15 && boardLevel == 0.0 && !grounded) {
            enc.reset()
            power = 0.0
            grounded = true
        }
        // if we are grounded and are moving down, stop
        if (power < 0) {
            power = if (grounded) 0.0 else 0.5*power
        }
        // power = power*13.3/voltage
        liftL.setPower(power)
        liftR.setPower(power)
    }

    fun onTarget(): Boolean {
        return (extension-levelToExtension(boardLevel)).absoluteValue < 0.1
    }

    private fun levelToExtension(l: Double): Double {
        return when (l) {
            0.0 -> -0.4
            else -> 2.48*l + 0.1
        }
    }
}
