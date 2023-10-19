package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware

class HangSubsystem(val hardware: Hardware) : Subsystem {
    enum class HangState {
        STOP,
        RAISE,
        LOWER,
    } var state = HangState.STOP
        private set
    // make private?
    var power = 0.0
        private set
    private var lastPower = 0.0

    init {
        hardware.hang.direction = DcMotorSimple.Direction.FORWARD
        // does this actually save any power lol, i don't think it does cause just back emf
        hardware.hang.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        hardware.hang.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        update(state)
    }

    override fun read() {}

    fun update(s: HangState) {
        state = s
        power = when (state) {
            HangState.STOP  ->  0.0
            HangState.RAISE ->  1.0
            HangState.LOWER -> -1.0
        }
    }

    override fun write() {
        // is there a cleaner way to write this?
        if (lastPower != power) {
            lastPower = power;
            hardware.hang.power = power;
        }
    }
}
