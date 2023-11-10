package org.firstinspires.ftc.teamcode.seventh.robot.node

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import com.scrapmetal.internode.NodeBroker
import com.scrapmetal.internode.Subsystem
import com.scrapmetal.internode.receive
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import com.scrapmetal.quackerama.hardware.QuackMotor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class LiftSubsystem(hardwareMap: HardwareMap) : Subsystem {
    override val topics: MutableMap<String, Any> = mutableMapOf(
            "lift/height" to 0.0,
            "lift/velocity" to 0.0,
            "lift/current" to 0.0,
            "lift/zeroed" to true,
    )
    private var height = 0.0
    private var zeroed = true
    private var velocity = 0.0
    private var current = 0.0
    private var power = 0.0
    override var updatePeriod = 1
    private val leftMotor: QuackMotor = QuackMotor(hardwareMap, "liftLeft")
    private val rightMotor: QuackMotor = QuackMotor(hardwareMap, "liftRight")
    private val enc: QuackQuadrature = QuackQuadrature(hardwareMap, "liftEnc", 1.0, 1.0)
    private val limit: TouchSensor = hardwareMap.get(TouchSensor::class.java, "liftLimit")

    init {
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD)
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD)
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        enc.setDirection(DcMotorSimple.Direction.FORWARD)
    }

    override fun read() {
        height = enc.getDist()
        velocity = enc.getLinearVelocity()
        current = leftMotor.getCurrent(CurrentUnit.AMPS) + rightMotor.getCurrent(CurrentUnit.AMPS)
        zeroed = limit.isPressed
    }

    override fun update(dt: Double) {
        // maybe add feedforward here because it's independent of feedback controls?
        // then why not add voltage compensation
        // what is the true scope of a subsystem?
        power = receive("lift/!power")
        publish("lift/height", height)
        publish("lift/velocity", velocity)
        publish("lift/current", current)
        publish("lift/zeroed", zeroed)
    }

    override fun write() {
        // i wonder how much time resetting takes, and if not resetting before height publish
        // can result in issues?
        if (zeroed && height != 0.0) enc.reset()
        leftMotor.setPower(power)
        rightMotor.setPower(power)
    }
}