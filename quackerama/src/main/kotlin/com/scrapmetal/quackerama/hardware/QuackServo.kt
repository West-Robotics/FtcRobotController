package com.scrapmetal.quackerama.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

/**
 * Servo wrapper with cached writes, built in servo pwm ranges, and less functions
 *
 * @param pwm servo model, used to determine pwm range
 * @param thresh minimum change in commanded position to necessitate a hardware write
 * @param usFrame framing rate in microseconds
 */
// TODO: add direction and initial position as part of the constructor to ensure that they are
//   explicitly stated
class QuackServo(
    hardwareMap: HardwareMap,
    name: String,
    pwm: ModelPWM,
    private var thresh: Double = 0.001,
    usFrame: Double = 5000.0
) {
    /**
     * PWM ranges for various servo models
     *
     * Axons are limited from 510-2490 to prevent accidental wraparound (may be adjusted) in the future
     */
    enum class ModelPWM(val min: Double, val max: Double) {
        AXON_MAX(510.0, 2490.0), AXON_MINI(510.0, 2490.0), AXON_MICRO(510.0, 2490.0),
        GOBILDA_TORQUE(500.0, 2500.0), GOBILDA_SPEED(500.0, 2500.0), GOBILDA_SUPER(500.0, 2500.0),
        GENERIC(500.0, 2500.0),
    }

    private val servo = hardwareMap.get(ServoImplEx::class.java, name)
    private var lastPosition = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max, usFrame)
    }

    fun setDirection(direction: Direction) {
        servo.direction = direction
    }

    fun setThresh(thresh: Double) {
        this.thresh = thresh
    }

    fun setPosition(position: Double) {
        if (abs(position - lastPosition) > thresh) {
            servo.position = position
            lastPosition = position
        }
    }

    /**
     * @param convertToPosition convert position argument to servo's 0-1 range
     */
    fun setPosition(position: Double, convertToPosition: (Double) -> Double) {
    	val convertedPosition = convertToPosition(position)
        if (abs(convertedPosition - lastPosition) > thresh) {
            servo.position = convertedPosition
            lastPosition = convertedPosition
        }
    }

    fun getCommandedPosition() = servo.position

}
