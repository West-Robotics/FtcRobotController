package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.Subsystem
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

object Robot {
    lateinit var hardwareMap: HardwareMap
    var voltage = 13.0
        private set
    /**
     * The time in ms since the last robot read
     */
    var dt = 0.0
        private set
    val timeSource = TimeSource.Monotonic
    var lastTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
    // private var voltageTimer = ElapsedTime()

    fun dtUpdate() {
        val currentTime = timeSource.markNow()
        dt = (currentTime - lastTime).toDouble(DurationUnit.MILLISECONDS)
        lastTime = currentTime
    }
    fun read(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.read()
        }
        // if (voltageTimer.seconds() > 5.0) {
        //     voltage = hardwareMap.voltageSensor.iterator().next().voltage
        //     voltageTimer.reset()
        // }
    }

    fun write(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.write()
        }
    }
}