package org.firstinspires.ftc.teamcode.seventh.robot.node

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.internode.NodeBroker
import com.scrapmetal.internode.Subsystem
import com.scrapmetal.internode.receive
import com.scrapmetal.quackerama.hardware.QuackServo

class EndEffectorSubsystem(hardwareMap: HardwareMap) : Subsystem {
    override val topics: MutableMap<String, Any> = mutableMapOf(
            "ee/commandedPitch" to 0.0,
    )
    private var commandedPitch = 0.0
    private var commandedLF = 0.0
    private var commandedRF = 0.0
    private var leftFilled = false
    private var rightFilled = false
    override var updatePeriod = 4
    // 25 kg blue servo
    private val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
    private val leftFinger = QuackServo(hardwareMap, "leftFinger", QuackServo.ModelPWM.GOBILDA_SPEED)
    private val rightFinger = QuackServo(hardwareMap, "rightFinger", QuackServo.ModelPWM.GOBILDA_SPEED)
    private val leftDetector = hardwareMap.get(ColorRangeSensor::class.java, "")

    init {
        pitch.setDirection(Servo.Direction.FORWARD)
        leftFinger.setDirection(Servo.Direction.FORWARD)
        rightFinger.setDirection(Servo.Direction.FORWARD)
    }

    override fun read() {
        commandedPitch = pitch.getCommandedPosition()
    }

    override fun update(dt: Double) {
        commandedPitch = receive("ee/!pitch")
        commandedLF = receive("ee/!lf")
        commandedRF = receive("ee/!rf")
        publish("ee/commandedPitch", commandedPitch)
    }

    override fun write() {
        pitch.setPosition(commandedPitch)
        leftFinger.setPosition(commandedLF)
        rightFinger.setPosition(commandedRF)
    }
}