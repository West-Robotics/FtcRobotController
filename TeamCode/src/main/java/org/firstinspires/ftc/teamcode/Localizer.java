package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface Localizer {
    Twist2dDual<Time> update();
    void startIMUThread();
}
