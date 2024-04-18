package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.hardware.QuackMotor


class HangSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class State(val power: Double) {
        STOP(0.0),
        RAISE(1.0),
        LOWER(-1.0),
    } private var state = State.STOP

    private val hang = QuackMotor(hardwareMap, "hang")

    init {
        hang.setDirection(DcMotorSimple.Direction.REVERSE)
        // i don't think this saves power
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        set(State.STOP)
    }

    override fun read() { }
    fun set(s: State) { state = s }
    override fun write() { hang.setPower(state.power) }
}
