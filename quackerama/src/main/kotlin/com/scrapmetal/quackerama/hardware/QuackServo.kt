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
 */
class QuackServo(hardwareMap: HardwareMap, name: String, pwm: ModelPWM, private var thresh: Double = 0.001) {
    enum class ModelPWM(val min: Double, val max: Double) {
        AXON_MAX(510.0, 2490.0), AXON_MINI(510.0, 2490.0), AXON_MICRO(510.0, 2490.0),
        GOBILDA_TORQUE(500.0, 2500.0), GOBILDA_SPEED(500.0, 2500.0), GOBILDA_SUPER(500.0, 2500.0),
        GENERIC(500.0, 2500.0),
    }

    private val servo = hardwareMap.get(ServoImplEx::class.java, name)
    private var lastPosition = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max)
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

    fun getCommandedPosition() = servo.position
}