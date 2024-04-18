package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackCRServo
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackServo

class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem {
    enum class State(val power: Double) {
        INTAKE(1.0),
        STACKTAKE(0.8),
        STOP(0.0),
        SPIT(-0.5),
    }
    private var power = 0.0
        private set
    private val STACK_HEIGHT = doubleArrayOf(0.19, 0.22, 0.25, 0.280, 0.297)

    var filledL = false
        private set
    var filledR = false
        private set
    private var rollerHeight: Int = 1
    private val intake = QuackMotor(hardwareMap, "intake")
    private val outerL = QuackServo(hardwareMap, "outerL", QuackServo.ModelPWM.GOBILDA_TORQUE, thresh = 0.0001)
    private val outerR = QuackServo(hardwareMap, "outerR", QuackServo.ModelPWM.GOBILDA_TORQUE, thresh = 0.0001)
    private val roller = QuackCRServo(hardwareMap, "roller", QuackCRServo.ModelPWM.CR_GOBILDA_SUPER, thresh = 0.0001)
    private val breakL = hardwareMap.get(DigitalChannel::class.java, "breakL")
    private val breakR = hardwareMap.get(DigitalChannel::class.java, "breakR")

    init {
        intake.setDirection(DcMotorSimple.Direction.FORWARD)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        outerL.setDirection(Servo.Direction.FORWARD)
        outerR.setDirection(Servo.Direction.REVERSE)
        roller.setDirection(DcMotorSimple.Direction.REVERSE)
        breakL.mode = DigitalChannel.Mode.INPUT
        breakR.mode = DigitalChannel.Mode.INPUT
    }

    fun raise() { rollerHeight = (rollerHeight+1).coerceAtMost(5) }
    fun lower() { rollerHeight = (rollerHeight-1).coerceAtLeast(1) }
    fun setHeight(h: Int) { rollerHeight = h.coerceIn(1..5) }

    // maybe add jam detection?
    override fun read() {
        filledL = !breakL.state
        filledR = !breakR.state
    }

    fun set(s: State) {
        power = s.power
        if (s == State.SPIT) setHeight(5)
    }
    // fun set(p: Double) { power = power }

    override fun write() {
        intake.setPower(power)
        roller.setPower(power)
        outerL.setPosition(STACK_HEIGHT[rollerHeight-1])
        outerR.setPosition(STACK_HEIGHT[rollerHeight-1])
    }
}
