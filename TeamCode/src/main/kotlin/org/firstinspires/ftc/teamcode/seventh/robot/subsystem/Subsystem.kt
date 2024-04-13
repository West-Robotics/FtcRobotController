package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.arcrobotics.ftclib.command.Subsystem

interface Subsystem : Subsystem {
    // WARNING: ORDER OF CALLING MATTERS
    // As indicated by https://docs.google.com/presentation/d/19RBNY5aCJK8FEnOWX4JIMRd9WVtqkeJ7pX1d8rqK2_U/edit#slide=id.g15295dfebf2_0_33
    // Latency (but not throughput) of commands is affected when dependencies are not called in order
    // The correct order is read() -> update() -> write()
    // Get fresh hardware data, do whatever you need to it, and finally write it to hardware
    // Does en masse vs per subsystem ordering matter?
    fun read()
    // <E extends Enum<E>> void update(E state) { }
    // fun update(enum: Enum<E>) { }
    fun write()
}
