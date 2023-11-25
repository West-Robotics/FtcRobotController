package org.firstinspires.ftc.teamcode.seventh.robot.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.Subsystem

object Robot {
    lateinit var hardwareMap: HardwareMap
    var voltage = 13.0
        private set
    private var voltageTimer = ElapsedTime()

    fun read(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.read()
        }
        if (voltageTimer.seconds() > 5.0) {
            voltage = hardwareMap.voltageSensor.iterator().next().voltage
            voltageTimer.reset()
        }
    }

    fun write(vararg subsystems: Subsystem) {
        for (s in subsystems) {
            s.write()
        }
    }
}