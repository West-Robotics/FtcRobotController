package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackCRServo
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var power = 0.0
        private set

    private val intake = QuackMotor(hardwareMap, "intake")
    private val outerLeft = QuackServo(hardwareMap, "outerLeft", QuackServo.ModelPWM.GOBILDA_TORQUE)
    private val outerRight = QuackServo(hardwareMap, "outerRight", QuackServo.ModelPWM.GOBILDA_TORQUE)
    private val roller = QuackCRServo(hardwareMap, "roller", QuackCRServo.ModelPWM.CR_GOBILDA_SUPER)
    private var rollerHeight: Int = 1
    fun raise() = when { rollerHeight != 5 -> rollerHeight++ else -> Any() }
    fun lower() = when { rollerHeight != 1 -> rollerHeight-- else -> Any() }
    fun setHeight(h: Int) {
        if (h in 1..5) {
            rollerHeight = h
        }
    }

    init {
        intake.setDirection(DcMotorSimple.Direction.REVERSE)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        outerLeft.setDirection(Servo.Direction.FORWARD)
        outerRight.setDirection(Servo.Direction.REVERSE)
        roller.setDirection(DcMotorSimple.Direction.REVERSE)
    }

    // maybe add jam detection?
    override fun read() {}

    fun update(s: RobotState) {
        when (s) {
            RobotState.INTAKE -> Pair(1.0, rollerHeight)
            RobotState.LOCK -> Pair(0.0, rollerHeight)
            RobotState.PRELOCK -> Pair(0.0, rollerHeight)
            RobotState.SPIT -> Pair(-0.4, 1)
            else -> Pair(power, rollerHeight)
        }.let {
            power = it.first
            setHeight(it.second)
        }
    }

    override fun write() {
        intake.setPower(power)
        roller.setPower(power)
        outerLeft.setPosition(Globals.STACK_HEIGHT[rollerHeight-1])
        outerRight.setPosition(Globals.STACK_HEIGHT[rollerHeight-1])
    }
}
