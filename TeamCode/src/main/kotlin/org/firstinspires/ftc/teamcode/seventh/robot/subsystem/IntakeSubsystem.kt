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

    val intake = QuackMotor(hardwareMap, "intake")
    val outerLeft = QuackServo(hardwareMap, "outerLeft", QuackServo.ModelPWM.GOBILDA_TORQUE)
    val outerRight = QuackServo(hardwareMap, "outerRight", QuackServo.ModelPWM.GOBILDA_TORQUE)
    val roller = QuackCRServo(hardwareMap, "roller", QuackCRServo.ModelPWM.CR_GOBILDA_SUPER)
    var rollerHeight: Int = 1
    fun raise() = when { rollerHeight != 5 -> rollerHeight++ else -> Any() }

    fun lower() = when { rollerHeight != 1 -> rollerHeight-- else -> Any() }

    init {
        intake.setDirection(DcMotorSimple.Direction.REVERSE)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        outerLeft.setDirection(Servo.Direction.FORWARD)
        outerRight.setDirection(Servo.Direction.REVERSE)
        roller.setDirection(DcMotorSimple.Direction.REVERSE)
    }

    // maybe add jam detection?
    override fun read() {}

    fun update(s: RobotState) {
        power = when (s) {
            RobotState.INTAKE -> 1.0
            RobotState.LOCK -> 0.0
            RobotState.SPIT -> -0.4
            else -> power
        }
    }

    override fun write() {
        intake.setPower(power)
        roller.setPower(power)
        outerLeft.setPosition(Globals.STACK_HEIGHT[rollerHeight-1])
        outerRight.setPosition(Globals.STACK_HEIGHT[rollerHeight-1])
    }
}
