package org.firstinspires.ftc.teamcode.clubfair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.clubfair.hardware.DriveClubfair;

import java.util.ArrayList;

@TeleOp(name = "Clubfair TeleOp")
public class TeleopClubfair extends OpMode {

    private DriveClubfair drive;
    private Controller controller;

    private static final int STORE_NUM = 4;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

    @Override
    public void init() {
        drive = new DriveClubfair(this, hardwareMap);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        drive.enableSquaredInputs();
    }

    @Override
    public void loop() {
        telemetry.update();
        controller.update();
        // invert right trigger so that unpressed is 1 and fully pressed is 0
        // math is gud
        // this is kinda weird ngl, a pain to use
        // double slow = (-1)*controller.right_trigger + 1;

        x.add(controller.left_stick_x);
        y.add(controller.left_stick_y);
        turn.add(controller.right_stick_x);

        // Remove
        if (x.size() > STORE_NUM) x.remove(0);
        if (y.size() > STORE_NUM) y.remove(0);
        if (turn.size() > STORE_NUM) turn.remove(0);

        double avgX = 0;
        for (int i = 0; i < x.size(); i++) avgX += x.get(i);
        avgX /= x.size();

        double avgY = 0;
        for (int i = 0; i < y.size(); i++) avgY += y.get(i);
        avgY /= y.size();

        double avgTurn = 0;
        for (int i = 0; i < turn.size(); i++) avgTurn += turn.get(i);
        avgTurn /= turn.size();

        drive.drive(avgX, avgY, avgTurn);
    }
}