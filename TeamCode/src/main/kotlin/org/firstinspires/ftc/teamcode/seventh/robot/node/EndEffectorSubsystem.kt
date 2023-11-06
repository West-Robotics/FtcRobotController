package org.firstinspires.ftc.teamcode.seventh.robot.node

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.architecture.NodeBroker
import com.scrapmetal.quackerama.architecture.Subsystem
import com.scrapmetal.quackerama.hardware.QuackServo

class EndEffectorSubsystem(hardwareMap: HardwareMap) : Subsystem {
    override val topics: MutableMap<String, Any> = mutableMapOf(
            "ee/commandedPitch" to 0.0,
    )
    private var commandedPitch = 0.0
    private var commandedLF = 0.0
    private var commandedRF = 0.0
    override var updatePeriod = 1
    // 25 kg blue servo
    private val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
    private val leftFinger = QuackServo(hardwareMap, "leftFinger", QuackServo.ModelPWM.GOBILDA_SPEED)
    private val rightFinger = QuackServo(hardwareMap, "rightFinger", QuackServo.ModelPWM.GOBILDA_SPEED)

    init {
        pitch.setDirection(Servo.Direction.FORWARD)
        leftFinger.setDirection(Servo.Direction.FORWARD)
        rightFinger.setDirection(Servo.Direction.FORWARD)
    }

    override fun read() {
        commandedPitch = pitch.getCommandedPosition()
    }

    override fun update(dt: Double) {
        commandedPitch = NodeBroker.topics["ee/!pitch"] as Double
        commandedLF = NodeBroker.topics["ee/!lf"] as Double
        commandedRF = NodeBroker.topics["ee/!rf"] as Double
        publish("ee/commandedPitch", commandedPitch)
    }

    override fun write() {
        pitch.setPosition(commandedPitch)
        leftFinger.setPosition(commandedLF)
        rightFinger.setPosition(commandedRF)
    }
}