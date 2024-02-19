package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.acmerobotics.roadrunner.util.epsilonEquals
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.controller.PDF
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.LIFT_HEIGHTS
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import kotlin.math.absoluteValue

class LiftSubsystem(hardwareMap: HardwareMap) : Subsystem {
    data class LiftState(
        var commandedExtension: Double = 0.0,
        var extension: Double = 0.0,
        var power: Double = 0.0,
        var current: Double = 0.0,
        var grounded: Boolean = true,
    ) val state = LiftState()
    private val liftPDF = PDF(Globals.LIFT_P, Globals.LIFT_D)
    var voltage = 13.3
    var reground = false

    private val liftLeft = QuackMotor(hardwareMap, "liftLeft")
    private val liftRight = QuackMotor(hardwareMap, "liftRight")
    private val enc = QuackQuadrature(
        hardwareMap,
        "liftLeft",
        8192.0, 1.0/Globals.LIFT_DISTANCE_PER_PULSE,
        DcMotorSimple.Direction.FORWARD,
    )

    init {
        // TODO: is brake strong enough?
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD)
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE)
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        enc.reset()
        update(-0.2, 0.0)
    }

    override fun read() {
        state.extension = enc.getDist()
        // only read current right when we need it (looptime destroyer)
        if (
            !state.grounded &&
            state.extension < 0.35 &&
            reground
        ) {
            state.current = liftLeft.getCurrent(CurrentUnit.AMPS) +
                            liftRight.getCurrent(CurrentUnit.AMPS)
        }
        // voltage = Robot.voltage
    }

    fun update(ce: Double, dt: Double, reground: Boolean = false) {
        this.reground = reground
        state.commandedExtension = ce
        if (!reground) {
            state.grounded = false
        }
        state.power = liftPDF.update(state.extension, state.commandedExtension, dt)
    }

    override fun write() {
        when {
            // reground if there's a current spike, we are low, want to be low, and are not reset
            state.current > 0.5 &&
            // state.current > state.lastCurrent &&
            state.extension < 0.35 &&
            reground &&
            !state.grounded
                -> {
                enc.reset()
                state.power = 0.0
                state.grounded = true
            }
            // if we are grounded and don't want to move up, brake
            state.grounded && state.power < 0.0 -> state.power = 0.0
        }

        state.power = state.power*13.3/voltage
        if (state.power < 0) state.power *= 0.5
        liftLeft.setPower(state.power)
        liftRight.setPower(state.power)
    }

    fun onTarget(): Boolean {
        return (state.extension-state.commandedExtension).absoluteValue < 0.2
    }
}
