package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

class OutputSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var armAng      = -120.0
    var pitchAng    = -45.0
    var leftAng     = Globals.FINGER_CLOSE
    var rightAng    = Globals.FINGER_CLOSE
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
        armLeft.setDirection(Servo.Direction.REVERSE)
        armRight.setDirection(Servo.Direction.FORWARD)
        pitch.setDirection(Servo.Direction.REVERSE)
        fingerLeft.setDirection(Servo.Direction.FORWARD)
        fingerRight.setDirection(Servo.Direction.REVERSE)
    }

    override fun read() {
        if (state == RobotState.INTAKE) {
            leftFilled = colorLeft.getDistance(DistanceUnit.MM) < 7.0
            rightFilled = colorRight.getDistance(DistanceUnit.MM) < 7.0
        }
    }

    fun update(s: RobotState, armAng: Double) {
        state = s
        when (s) {
            RobotState.LOCK     -> Triple(-49.0,         Globals.FINGER_CLOSE, Globals.FINGER_CLOSE)
            RobotState.INTAKE   -> Triple(-48.0,         Globals.FINGER_OPEN,  Globals.FINGER_OPEN)
            RobotState.SPIT     -> Triple(-48.0,         Globals.FINGER_OPEN,  Globals.FINGER_OPEN)
            RobotState.BACKDROP -> Triple(-49.0,         Globals.FINGER_CLOSE, Globals.FINGER_CLOSE)
            RobotState.EXTEND   -> Triple(-75.0,         Globals.FINGER_CLOSE, Globals.FINGER_CLOSE)
            RobotState.SCORE    -> Triple(-75.0,         Globals.FINGER_OPEN,  Globals.FINGER_OPEN)
            RobotState.SCORE_L  -> Triple(-75.0,         Globals.FINGER_OPEN,  Globals.FINGER_CLOSE)
            RobotState.SCORE_R  -> Triple(-75.0,         Globals.FINGER_CLOSE, Globals.FINGER_OPEN)
            // RobotState.SCORE_R  -> Triple(-armAng-120.0, Globals.FINGER_CLOSE, Globals.FINGER_OPEN)
            RobotState.GROUND   -> Triple(0.0,           Globals.FINGER_CLOSE, Globals.FINGER_CLOSE)
        }.let {
            // this.armAng = armAng
            this.armAng = when(state) {
                RobotState.EXTEND, RobotState.SCORE, RobotState.SCORE_L, RobotState.SCORE_R -> -30.0
                RobotState.LOCK, RobotState.BACKDROP -> -125.0
                RobotState.INTAKE, RobotState.SPIT -> -130.0
                else -> -120.0
            }
            pitchAng = it.first
            leftAng = it.second
            rightAng = it.third
        }
    }

    override fun write() {
        armLeft.setPosition(armAng)  { theta: Double -> theta/180.98 + 1.0 }
        armRight.setPosition(armAng) { theta: Double -> theta/180.98 + 1.0 }
        pitch.setPosition(pitchAng)  { theta: Double -> theta/270.0 + 0.5 }
        fingerLeft.setPosition(leftAng)
        fingerRight.setPosition(rightAng)
    }
}
