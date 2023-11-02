package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware

class IntakeSubsystem(val hardware: Hardware) : Subsystem {
    enum class IntakeState {
        INTAKE,
        STOP,
        SPIT,
        ;

        // ruh roh
        var rollerHeight: Int = 1
        fun raise() {
            when {
                rollerHeight != 5 -> rollerHeight++
            }
        }

        fun lower() {
            when {
                rollerHeight != 1 -> rollerHeight--
            }
        }
    } var state = IntakeState.STOP
        private set

    var power = 0.0
        private set
    private var lastPower = 0.0
//    private double lastAngle = 0
//    private double angle = 0

    init {
        hardware.intake.direction = DcMotorSimple.Direction.REVERSE
        hardware.intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        hardware.intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        hardware.outerPivotLeft.setDirection(Servo.Direction.FORWARD)
//        hardware.outerPivotLeft.setPwmRange(new PwmControl.PwmRange(500, 2500))
//        hardware.outerPivotRight.setDirection(Servo.Direction.REVERSE)
//        hardware.outerPivotRight.setPwmRange(new PwmControl.PwmRange(500, 2500))
        update(state)
    }

    // maybe add jam detection?
    override fun read() {}

    fun update(s: IntakeState) {
        power = when (s) {
            IntakeState.INTAKE -> 1.0
            IntakeState.STOP -> 0.0
            IntakeState.SPIT -> -0.4
        }
        write()
    }

    override fun write() {
        // maybe a nice wrapper?
        if (lastPower != power) {
            lastPower = power
            hardware.intake.power = power
        }
//        if (lastAngle != a) {
//            lastAngle = a
//            hardware.outerPivotLeft.setPosition(a)
//            hardware.outerPivotRight.setPosition(a)
//        }
    }
}
