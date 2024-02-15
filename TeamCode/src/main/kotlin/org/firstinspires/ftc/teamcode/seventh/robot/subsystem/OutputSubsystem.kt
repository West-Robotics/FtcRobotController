package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackAnalog
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.FINGER_CLOSE
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.FINGER_OPEN
import kotlin.math.ulp

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
    var curArmAngLeft = 0.0
        private set
    var curArmAngRight = 0.0
        private set
    var curArmAng = 0.0
        private set

    private val armLeft = QuackServo(hardwareMap, "armLeft", QuackServo.ModelPWM.AXON_MAX)
    private val armRight = QuackServo(hardwareMap, "armRight", QuackServo.ModelPWM.AXON_MAX)
    private val armEncLeft = QuackAnalog(hardwareMap, "armEncLeft")
    private val armEncRight = QuackAnalog(hardwareMap, "armEncRight")
    private val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
    private val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.AXON_MICRO)
    private val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.AXON_MICRO)
    private val colorLeft = hardwareMap.get(ColorRangeSensor::class.java, "colorLeft")
    private val colorRight = hardwareMap.get(ColorRangeSensor::class.java, "colorRight")
    var pivotOffset = 0.0

    init {
        armLeft.setDirection(Servo.Direction.REVERSE)
        armRight.setDirection(Servo.Direction.FORWARD)
        pitch.setDirection(Servo.Direction.REVERSE)
        fingerLeft.setDirection(Servo.Direction.REVERSE)
        fingerRight.setDirection(Servo.Direction.FORWARD)
        pitch.setPosition(60.0)
        fingerLeft.setPosition(FINGER_CLOSE)
        fingerRight.setPosition(FINGER_CLOSE)
    }

    override fun read() {
        if (robotState == RobotState.INTAKE && !Globals.AUTO) {
            leftFilled = colorLeft.getDistance(DistanceUnit.MM) < 14.0
            rightFilled = colorRight.getDistance(DistanceUnit.MM) < 14.0
        }
        curArmAngLeft = armEncLeft.getRawVoltage()
        curArmAngRight = 3.3 - armEncRight.getRawVoltage()
        curArmAng = -109.14*(curArmAngLeft + curArmAngRight)/2 + 88.78
    }

    fun update(s: RobotState, armAng: Double) {
        robotState = s
        outState = when (s) {
            RobotState.LOCK     -> OutputState(armAng, 65.0+pivotOffset, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.PRELOCK  -> OutputState(armAng, 62.0+pivotOffset, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.INTAKE   -> OutputState(armAng, 62.0+pivotOffset, FINGER_OPEN, FINGER_OPEN)
            RobotState.SPIT     -> OutputState(armAng, 62.0+pivotOffset, FINGER_OPEN, FINGER_OPEN)
            RobotState.ALIGN    -> OutputState(armAng, 65.0+pivotOffset, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.BACKDROP -> OutputState(armAng, 65.0+pivotOffset, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.EXTEND   -> OutputState(armAng, armAng+105.0+pivotOffset, FINGER_CLOSE, FINGER_CLOSE)
            RobotState.SCORE    -> OutputState(armAng, armAng+105.0+pivotOffset, FINGER_OPEN, FINGER_OPEN)
            RobotState.SCORE_L  -> OutputState(armAng, armAng+105.0+pivotOffset, FINGER_OPEN, FINGER_CLOSE)
            RobotState.SCORE_R  -> OutputState(armAng, armAng+105.0+pivotOffset, FINGER_CLOSE, FINGER_OPEN)
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
