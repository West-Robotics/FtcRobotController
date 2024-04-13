package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo


class DroneSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class State(val position: Double) {
        LODED(0.15),
        DIPER(0.0),
        EQUILI(0.10),
    } private var state = State.LODED

    private val drone = QuackServo(hardwareMap, "drone", QuackServo.ModelPWM.GOBILDA_TORQUE)

    init {
        drone.setDirection(Servo.Direction.FORWARD)
        write()
    }

    override fun read() { }
    fun set(s: State) { state = s }
    override fun write() { drone.setPosition(state.position) }
}