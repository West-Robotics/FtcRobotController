package com.scrapmetal.quackerama.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

/**
 * Motor wrapper with cached writes and less functions
 *
 * @param thresh minimum change in commanded power to necessitate a hardware write
 */
class QuackMotor(val hardwareMap: HardwareMap, name: String, private var thresh: Double = 0.005) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, name)
    private var lastPower = 0.0

    init { motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {
        motor.zeroPowerBehavior = zeroPowerBehavior
    }

    fun setDirection(direction: Direction) { motor.direction = direction }

    fun setPower(power: Double) {
        if (abs(power - lastPower) > thresh) {
            motor.power = power
            lastPower = power
        }
    }

    fun setThresh(thresh: Double) { this.thresh = thresh }

    fun setCurrentAlert(current: Double) { motor.setCurrentAlert(current, CurrentUnit.AMPS) }
    fun getCurrentAlert(current: Double) = motor.getCurrentAlert(CurrentUnit.AMPS)
    fun getCurrent(currentUnit: CurrentUnit) = motor.getCurrent(currentUnit)
    fun isOverCurrent() = motor.isOverCurrent
}