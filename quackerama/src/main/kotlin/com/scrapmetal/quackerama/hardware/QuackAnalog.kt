package com.scrapmetal.quackerama.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Motor encoder wrapper with various QOL functions and built-in velocity overflow correction
 * WARNING: not ready for usage yet
 */
class QuackAnalog(hardwareMap: HardwareMap, name: String, val transform: (Double) -> Double = { x: Double -> x }) {
    private val analog = hardwareMap.analogInput.get(name)
    private var offset = 0.0
    private var inverted = false

    /**
     * Get scaled value from 0-1
     */
    fun getValue() = if (!inverted) analog.voltage/3.3 else (3.3-analog.voltage)/3.3

    /**
     * Get transformed scaled value
     */
    fun getTransformedValue() = transform(getValue())
    fun getRawVoltage() = analog.voltage

    fun invert() { inverted = true }
    fun uninvert() { inverted = false }
}