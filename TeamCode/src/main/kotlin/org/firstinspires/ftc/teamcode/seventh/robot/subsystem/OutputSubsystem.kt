package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.FINGER_CLOSE
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.FINGER_OPEN

class OutputSubsystem(hardwareMap: HardwareMap) : Subsystem {
    data class OutputState(
        val arm: Double = -120.0,
        val pitch: Double = -58.0,
        val left: Double = FINGER_CLOSE,
        val right: Double = FINGER_CLOSE,
    ) var outState = OutputState()
        private set
    private var robotState = RobotState.LOCK

    var leftFilled = false
        private set
    var rightFilled = false
        private set

    private val armLeft = QuackServo(hardwareMap, "armLeft", QuackServo.ModelPWM.AXON_MAX)
    private val armRight = QuackServo(hardwareMap, "armRight", QuackServo.ModelPWM.AXON_MAX)
    private val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
    private val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.GOBILDA_SPEED)
    private val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.GOBILDA_SPEED)
    private val colorLeft = hardwareMap.get(ColorRangeSensor::class.java, "colorLeft")
    private val colorRight = hardwareMap.get(ColorRangeSensor::class.java, "colorRight")

    init {
        armLeft.setDirection(Servo.Direction.REVERSE)
        armRight.setDirection(Servo.Direction.FORWARD)
        pitch.setDirection(Servo.Direction.REVERSE)
        fingerLeft.setDirection(Servo.Direction.FORWARD)
        fingerRight.setDirection(Servo.Direction.REVERSE)
    }

    override fun read() {
        if (robotState == RobotState.INTAKE) {
            leftFilled = colorLeft.getDistance(DistanceUnit.MM) < 7.0
            rightFilled = colorRight.getDistance(DistanceUnit.MM) < 7.0
        }
        // curArmAng = axon stuff
    }

    fun update(s: RobotState, armAng: Double) {
        robotState = s
        outState = when (s) {
            RobotState.LOCK     -> OutputState(armAng, 58.0, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.INTAKE   -> OutputState(armAng, 48.0, FINGER_OPEN, FINGER_OPEN)
            RobotState.SPIT     -> OutputState(armAng, 48.0, FINGER_OPEN, FINGER_OPEN)
            RobotState.BACKDROP -> OutputState(armAng, 58.0, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.EXTEND   -> OutputState(armAng, armAng+120.0, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.SCORE    -> OutputState(armAng, 75.0, FINGER_OPEN, FINGER_OPEN)
            RobotState.SCORE_L  -> OutputState(armAng, 75.0, FINGER_OPEN, FINGER_CLOSE)
            RobotState.SCORE_R  -> OutputState(armAng, 75.0, FINGER_CLOSE, FINGER_OPEN)
            // RobotState.SCORE_R  -> OutputState(-armAng-120.0, Globals.FINGER_CLOSE, Globals.FINGER_OPEN)
        }
    }

    override fun write() {
        armLeft.setPosition(outState.arm)  { theta: Double -> theta/180.0 + 1.0 }
        armRight.setPosition(outState.arm) { theta: Double -> theta/180.0 + 1.0 }
        pitch.setPosition(outState.pitch)  { theta: Double -> theta/270.0 }
        fingerLeft.setPosition(outState.left)
        fingerRight.setPosition(outState.right)
    }
}
