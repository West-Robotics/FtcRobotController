package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

public abstract class Subsystem {
    public <E extends Enum<E>> void update(Class<E> stateEnum, E state) { }
    public <E extends Enum<E>> void read() { }
    public <E extends Enum<E>> void write() { }
}
