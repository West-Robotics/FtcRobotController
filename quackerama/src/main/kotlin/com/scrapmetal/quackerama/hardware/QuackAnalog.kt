package com.scrapmetal.quackerama.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Motor encoder wrapper with various QOL functions and built-in velocity overflow correction
 * WARNING: not ready for usage yet
 */
class QuackAnalog(hardwareMap: HardwareMap, name: String) {
    private val analog = hardwareMap.get(AnalogInput::class.java, name)
    private var dir: Direction = Direction.FORWARD
    private var offset = 0.0

    fun setDirection(direction: Direction) {
        dir = direction
    }

    fun getTicks() = when (dir) { Direction.FORWARD -> 1; Direction.REVERSE -> -1}
                     .let { it*(getValue() - offset) }

    fun getAng() = getValue()*2*kotlin.math.PI
    fun getValue() = analog.voltage/3.3
    fun getRawVoltage() = analog.voltage


    /**
     * "Reset" encoder without using STOP_AND_RESET_ENCODER
     */
    fun reset() {
        offset = getValue()
    }
}