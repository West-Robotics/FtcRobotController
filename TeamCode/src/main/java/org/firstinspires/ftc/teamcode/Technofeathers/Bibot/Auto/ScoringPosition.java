/*package org.firstinspires.ftc.teamcode.Technofeathers.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Technofeathers.Teleop.EggnogTeleopAutomated1Controller;

public class ScoringPosition extends EggnogTeleopAutomated1Controller{
    public void PixelScoringPosition() throws InterruptedException {
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while (e.seconds() < 0.5) {
            grabber.setPosition(0.67);
        }
        while (e.seconds() < 1.7) {
            lift1.setPower(1);
            lift2.setPower(1);
        }
        while (1.7 < e.seconds() && e.seconds() < 2.7) {
            pivot1.setPosition(0);
        }
    }
}

 */
