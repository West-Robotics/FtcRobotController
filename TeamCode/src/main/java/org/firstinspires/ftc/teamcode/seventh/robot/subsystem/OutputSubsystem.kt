package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware

class OutputSubsystem(val hardware: Hardware) : Subsystem {
    enum class OutputState {
        INTAKE,
        LOCK,
        INTERMEDIARY,
        READY,
        DROP,
        DROP_L,
        DROP_R,
        PLOP_READY,
        PLOP_L,
        PLOP_R,
        POOP,
    } var state = OutputState.LOCK
        private set

    var pivAng          = 0.0
    var leftAng         = 0.0
    var rightAng        = 0.0
    var lastPivotAngle  = 0.0
    var lastLeftAngle   = 0.0
    var lastRightAngle  = 0.0

    init {
        // this doesn't work cause blue can't be programmed lol
        // hardware.pivot.setDirection(Servo.Direction.FORWARD)
        hardware.pivot.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        hardware.fingerLeft.direction = Servo.Direction.FORWARD
        hardware.fingerLeft.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        hardware.fingerRight.direction = Servo.Direction.FORWARD
        hardware.fingerRight.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        update(state)
    }

    override fun read() {}

    fun update(s: OutputState) {
        when (s) {
            OutputState.INTAKE          -> Triple(Globals.PIVOT_INTAKE,         Globals.FINGER_L_OPEN,  Globals.FINGER_R_OPEN)
            OutputState.LOCK            -> Triple(Globals.PIVOT_INTAKE,         Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            OutputState.INTERMEDIARY    -> Triple(Globals.PIVOT_INTERMEDIARY,   Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            OutputState.READY           -> Triple(Globals.PIVOT_OUTTAKE,        Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            OutputState.DROP            -> Triple(Globals.PIVOT_OUTTAKE,        Globals.FINGER_L_OPEN,  Globals.FINGER_R_OPEN)
            OutputState.DROP_L          -> Triple(Globals.PIVOT_OUTTAKE,        Globals.FINGER_L_OPEN,  Globals.FINGER_R_CLOSE)
            OutputState.DROP_R          -> Triple(Globals.PIVOT_OUTTAKE,        Globals.FINGER_L_CLOSE, Globals.FINGER_R_OPEN)
            OutputState.PLOP_READY      -> Triple(Globals.PIVOT_PLOP,           Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
            OutputState.PLOP_L          -> Triple(Globals.PIVOT_PLOP,           Globals.FINGER_L_OPEN,  Globals.FINGER_R_CLOSE)
            OutputState.PLOP_R          -> Triple(Globals.PIVOT_PLOP,           Globals.FINGER_L_CLOSE, Globals.FINGER_R_OPEN)
            OutputState.POOP            -> Triple(Globals.PIVOT_POOP,           Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE)
        }.let {
            pivAng = it.first
            leftAng = it.second
            rightAng = it.third
        }
    }

    override fun write() {
        // TODO: add small tolerances because imperfections
        if (pivAng != lastPivotAngle) {
            lastPivotAngle = pivAng
            hardware.pivot.position = pivAng
        }
        if (leftAng != lastLeftAngle) {
            lastLeftAngle = leftAng
            hardware.fingerLeft.position = leftAng
        }
        if (rightAng != lastRightAngle) {
            lastRightAngle = rightAng
            hardware.fingerRight.position = rightAng
        }
    }
}
