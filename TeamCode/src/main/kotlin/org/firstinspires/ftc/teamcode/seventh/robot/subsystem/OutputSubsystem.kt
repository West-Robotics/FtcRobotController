package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.controller.AsymTrapezoidMP
import com.scrapmetal.quackerama.hardware.QuackAnalog
import com.scrapmetal.quackerama.hardware.QuackServo

class OutputSubsystem(hardwareMap: HardwareMap) : Subsystem {
    data class State(
        var arm: Arm = Arm.IN,
        var yaw: Double = 0.0,
        var pitch: Pitch = Pitch.IN,
        var roll: Roll = Roll.HORIZ,
        var claw: Claw = Claw.BOTH,
    ) var state = State()

    enum class Arm(val angle: Double) {
        IN(180+50.0),
        INTAKE(180+45.0),
        PULLOUT(180+50.0),
        BACKDROP(5.0),
        HIGH(45.0),
        GROUND(-30.0),
    }
    enum class Pitch(val angle: Double) {
        IN(60.0),
        INTAKE(50.0),
        PULLOUT(65.0),
        OUT(-5.0),
    }
    enum class Roll(val angle: Double) {
        /**
         * LR
         */
        HORIZ(0.0),
        /**
         *    R
         *
         * L
         */
        // 53
        LM(53.0),
        /**
         * L
         *
         *    R
         */
        RM(-53.0),
        /**
         * R
         *
         * L
         */
        VERT(90.0),
        // /**
        //  * Inversion of HORIZ
        //  */
        // HORIZ_I(180.0),
        /**
         * Inversion of LM
         */
        LM_I(-120.0),
        /**
         * Inversion of RM
         */
        RM_I(120.0),
        /**
         * Inversion of VERT
         */
        VERT_I(-90.0),
    }
    /**
     * Describes which sides of the claw holds a pixel
     */
    enum class Claw(val left: Double, val right: Double) {
        BOTH(0.01, 0.01),
        LEFT(0.01, 0.7),
        RIGHT(0.7, 0.01),
        NONE(0.7, 0.7),
    }

    private var armMP = AsymTrapezoidMP(Arm.IN.angle, Arm.IN.angle, 0.0, 0.0, 0.0)
    private var armTimer = ElapsedTime()
    private val armL = QuackServo(hardwareMap, "armL", QuackServo.ModelPWM.AXON_MAX)
    private val armR = QuackServo(hardwareMap, "armR", QuackServo.ModelPWM.AXON_MAX)
    private val endL = QuackServo(hardwareMap, "endL", QuackServo.ModelPWM.AXON_MICRO)
    private val endR = QuackServo(hardwareMap, "endR", QuackServo.ModelPWM.AXON_MICRO)
    private val fingerL = QuackServo(hardwareMap, "fingerL", QuackServo.ModelPWM.AXON_MICRO)
    private val fingerR = QuackServo(hardwareMap, "fingerR", QuackServo.ModelPWM.AXON_MICRO)
    private val endLEnc = QuackAnalog(hardwareMap, "endLEnc") { x: Double -> 180 * x }
    private val endREnc = QuackAnalog(hardwareMap, "endREnc") { x: Double -> 180 * x }
    // private val endLAngOffset = -180.0
    // private val endRAngOffset = -135.0
    private val endLAngOffset = 0.0
    private val endRAngOffset = 0.0
    var endLAng = 0.0
        private set
    var endRAng = 0.0
        private set

    init {
        armL.setDirection(Servo.Direction.REVERSE)
        armR.setDirection(Servo.Direction.FORWARD)
        endL.setDirection(Servo.Direction.FORWARD)
        endR.setDirection(Servo.Direction.REVERSE)
        fingerL.setDirection(Servo.Direction.REVERSE)
        fingerR.setDirection(Servo.Direction.FORWARD)
        endLEnc.invert()
    }

    override fun read() {
        endLAng = endLEnc.getTransformedValue() + endLAngOffset
        endRAng = endREnc.getTransformedValue() + endRAngOffset
    }

    fun set(s: State) {
        if (s.arm != state.arm) {
            armMP = AsymTrapezoidMP(state.arm.angle, s.arm.angle, 5000.0, -1000.0, 4000.0)
            armTimer.reset()
        }
        state = s
    }
    fun set(a: Arm) {
        if (a != state.arm) {
            armMP = AsymTrapezoidMP(state.arm.angle, a.angle, 5000.0, -1000.0, 4000.0)
            armTimer.reset()
        }
        state.arm = a
    }
    fun set(y: Double) { state.yaw = y }
    fun set(p: Pitch) { state.pitch = p }
    fun set(r: Roll) { state.roll = r }
    fun set(c: Claw) { state.claw = c }

    override fun write() {
        armL.setPosition(armMP.update(armTimer.seconds()).s + state.yaw.coerceIn(-45.0..45.0)) { a: Double -> a*(1/355.0) + 90/355.0 }
        armR.setPosition(armMP.update(armTimer.seconds()).s - state.yaw.coerceIn(-45.0..45.0)) { a: Double -> a*(1/355.0) + 90/355.0 }
        endL.setPosition(state.pitch.angle - state.roll.angle) { theta: Double -> theta/200.0 + 120/200.0 }
        endR.setPosition(state.pitch.angle + state.roll.angle) { theta: Double -> theta/200.0 + 120/200.0 }
        fingerL.setPosition(state.claw.left)
        fingerR.setPosition(state.claw.right)
    }
}
