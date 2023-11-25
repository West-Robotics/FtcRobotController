package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.hardware.QuackMotor


class HangSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class HangState {
        STOP,
        RAISE,
        LOWER,
    } var state = HangState.STOP
        private set
    // make private?
    var power = 0.0
        private set
    private var lastPower = 0.0

    val hang = QuackMotor(hardwareMap, "hang")

    init {
        hang.setDirection(DcMotorSimple.Direction.FORWARD)
        // i don't think this saves power
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        update(state)
    }

    override fun read() {}

    fun update(s: HangState) {
        state = s
        power = when (state) {
            HangState.STOP  ->  0.0
            HangState.RAISE ->  1.0
            HangState.LOWER -> -1.0
        }
    }

    override fun write() {
        hang.setPower(power)
    }
}
