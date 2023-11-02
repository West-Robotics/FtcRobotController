package com.scrapmetal.quackerama.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap

class QuackEnc(hardwareMap: HardwareMap, name: String, private var ticksPerRev: Double, private var ticksPerDist: Double) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, name)
    private var dir: Direction = Direction.FORWARD
    private var offset = 0

    fun setTicksPerDist(tpd: Double) {
        ticksPerDist = tpd
    }

    fun setTicksPerRev(tpr: Double) {
        ticksPerRev = tpr
    }

    fun setDirection(direction: Direction) {
        dir = direction
    }

    fun getDist() = getTicks()/ticksPerDist

    fun getRevs() = getTicks()/ticksPerRev

    fun getTicks() = when (dir) { Direction.FORWARD -> 1; Direction.REVERSE -> -1}
                     .let { it*(motor.currentPosition - offset) }

    fun reset() {
        offset = motor.currentPosition
    }
}