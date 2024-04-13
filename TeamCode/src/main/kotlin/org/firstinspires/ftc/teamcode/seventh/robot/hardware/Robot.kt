package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.Subsystem
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

object Robot {
    private lateinit var hardwareMap: HardwareMap
    private lateinit var telemetry: Telemetry
    private lateinit var CONTROL_HUB: LynxModule
    /**
     * The time in seconds since the last robot read
     */
    private var dt = 0.0
    private val timeSource = TimeSource.Monotonic
    private var lastTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
    private var voltageTimer = ElapsedTime()
    var voltage = 13.3
        private set

    private var gigapad1: Gigapad? = null
    private var gigapad2: Gigapad? = null

    fun init(
        hardwareMap: HardwareMap,
        telemetry: Telemetry,
        gigapad1: Gigapad?,
        gigapad2: Gigapad?,
    ) {
        this.hardwareMap = hardwareMap
        this.telemetry = telemetry
        this.gigapad1 = gigapad1
        this.gigapad2 = gigapad2
        CommandScheduler.getInstance().reset()
        initHubs()
    }

    private fun initHubs() {
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
    }

    fun read(vararg subsystems: Subsystem) {
        CONTROL_HUB.clearBulkCache()
        dtUpdate()
        for (s in subsystems) {
            s.read()
        }
        if (voltageTimer.seconds() > 10.0) {
            voltage = hardwareMap.voltageSensor.iterator().next().voltage
            voltageTimer.reset()
        }
        gigapad1?.readButtons()
        gigapad2?.readButtons()
        CommandScheduler.getInstance().run()
    }

    fun write(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.write()
        }
        telemetry.update()
    }

    fun getHwMap() = hardwareMap

    fun getDt() = dt

    fun dtUpdate() {
        val currentTime = timeSource.markNow()
        dt = (currentTime - lastTime).toDouble(DurationUnit.SECONDS)
        lastTime = currentTime
    }
}