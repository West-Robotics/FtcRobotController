package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackServo


class DroneSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class DroneState {
        LODED,
        DIPER,
    }
    var position = 0.0
        private set

    private val drone = QuackServo(hardwareMap, "drone", QuackServo.ModelPWM.GOBILDA_TORQUE)

    init {
        drone.setDirection(Servo.Direction.FORWARD)
        drone.setPosition(0.0)
    }

    override fun read() {}

    fun update(s: DroneState) {
        position = when (s) {
            DroneState.DIPER -> 0.0
            DroneState.LODED -> 0.15
        }
    }

    override fun write() {
        drone.setPosition(position)
    }
}