package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware

class LiftSubsystem(val hardware: Hardware) : Subsystem {
    enum class LiftState {
        DOWN,
        UP,
    } var state = LiftState.DOWN
        private set

    var distance = 0.0
        private set
    var power = 0.0
        private set
    private var lastPower = 0.0
    var pressed = true
        private set
    private val liftPid = PIDController(Globals.LIFT_P, Globals.LIFT_I, Globals.LIFT_D)
    var voltage = 13.0

    init {
        hardware.liftLeft.direction = DcMotorSimple.Direction.FORWARD
        hardware.liftLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hardware.liftLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardware.liftRight.direction = DcMotorSimple.Direction.REVERSE
        hardware.liftRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hardware.liftRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardware.liftLeftEnc.setDirection(Motor.Direction.FORWARD)
        hardware.liftLeftEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE)
        hardware.liftLeftEnc.reset()
        hardware.liftRightEnc.setDirection(Motor.Direction.FORWARD)
        hardware.liftRightEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE)
        hardware.liftRightEnc.reset()
        liftPid.setOutputRange(0.0, 0.7)
        liftPid.setTolerance(5.0)
        liftPid.reset()
        liftPid.enable()
        update(state, -1)
    }

    override fun read() {
        distance = hardware.liftLeftEnc.distance
        pressed = hardware.limit.isPressed
        voltage = hardware.voltage
    }

    fun update(s: LiftState, height: Int) {
        state = s
        liftPid.setpoint = when (s) {
            LiftState.UP    -> Globals.LIFT_HEIGHTS[height]
            LiftState.DOWN  -> Globals.LIFT_MIN
        }
        power = liftPid.performPID(distance)
        write()
    }

    override fun write() {
        // can this be cleaner?
        if (pressed) {
            if (distance != 0.0) {
                hardware.liftLeftEnc.reset()
            }
            if (power < 0.0) {
                power = 0.0
            }
        } else if (liftPid.onTarget()) {
            power = 0.0
        } else if (power < 0.0) {
            power /= if (distance >= 1.0) {
                4.0
            } else {
                2.0
            }
        }
        power = power*13.0/hardware.voltage
        if (lastPower != power) {
            lastPower = power
            hardware.liftLeft.power = power
            hardware.liftRight.power = power
        }
    }
}
