package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

class OutputSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var armAng      = 0.0
    var pitchAng    = 0.0
    var leftAng     = 0.0
    var rightAng    = 0.0
    var leftFilled  = false
    var rightFilled = false

    val armLeft = QuackServo(hardwareMap, "armLeft", QuackServo.ModelPWM.AXON_MAX)
    val armRight = QuackServo(hardwareMap, "armRight", QuackServo.ModelPWM.AXON_MAX)
    val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
    val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.GOBILDA_SPEED)
    val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.GOBILDA_SPEED)
    val colorLeft = hardwareMap.get(ColorRangeSensor::class.java, "colorLeft")
    val colorRight = hardwareMap.get(ColorRangeSensor::class.java, "colorRight")

    var state = RobotState.LOCK

    init {
        armLeft.setDirection(Servo.Direction.FORWARD)
        armRight.setDirection(Servo.Direction.REVERSE)
        pitch.setDirection(Servo.Direction.REVERSE)
        fingerLeft.setDirection(Servo.Direction.FORWARD)
        fingerRight.setDirection(Servo.Direction.REVERSE)
    }

    override fun read() {
        if (state == RobotState.INTAKE) {
            leftFilled = colorLeft.getDistance(DistanceUnit.MM) < 20.0
            rightFilled = colorRight.getDistance(DistanceUnit.MM) < 20.0
        }
    }

    fun update(s: RobotState, armAng: Double) {
        state = s
        when (s) {
            RobotState.LOCK     -> Triple(-45.0,         Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            RobotState.INTAKE   -> Triple(-45.0,         Globals.FINGER_L_OPEN,  Globals.FINGER_R_OPEN)
            RobotState.SPIT     -> Triple(-45.0,         Globals.FINGER_L_OPEN,  Globals.FINGER_R_OPEN)
            RobotState.BACKDROP -> Triple(-30.0,         Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            RobotState.EXTEND   -> Triple(-armAng-120.0, Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            RobotState.SCORE    -> Triple(-armAng-120.0, Globals.FINGER_L_OPEN,  Globals.FINGER_R_OPEN)
            RobotState.SCORE_L  -> Triple(-armAng-120.0, Globals.FINGER_L_OPEN,  Globals.FINGER_R_CLOSE)
            RobotState.SCORE_R  -> Triple(-armAng-120.0, Globals.FINGER_L_CLOSE, Globals.FINGER_R_OPEN)
            RobotState.GROUND   -> Triple(0.0,           Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
        }.let {
            this.armAng = armAng
            pitchAng = it.first
            leftAng = it.second
            rightAng = it.third
        }
    }

    override fun write() {
        armLeft.setPosition(armAng)  { theta: Double -> theta/355.0 + 1.0 }
        armRight.setPosition(armAng) { theta: Double -> theta/355.0 + 1.0 }
        pitch.setPosition(pitchAng)  { theta: Double -> theta/270.0 + 0.5 }
        fingerLeft.setPosition(leftAng)
        fingerRight.setPosition(rightAng)
    }
}
