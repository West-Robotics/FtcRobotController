package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Balloon Mode")
public class AutoBolon extends LinearOpMode{

    private org.firstinspires.ftc.teamcode.slowBolon.CamBolon Cam;
    private int DONDEESTAELDUCKY;
    private DriveBolon d;

    double distance;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Cam = new org.firstinspires.ftc.teamcode.slowBolon.CamBolon();
        Cam.init(hardwareMap);

        hardwareMap.get(DcMotor.class,"frontRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotor.class,"frontRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d = new DriveBolon(this,hardwareMap);
        DONDEESTAELDUCKY = Cam.getspot();
        ElapsedTime runtime = new ElapsedTime();

        //telemetry.addData("where",DONDEESTAELDUCKY);
        telemetry.addData("leastduckydiff", Cam.pipeline.leastduckydiff);
        telemetry.addData("leasttapediff", Cam.pipeline.leasttapediff);
        telemetry.addData("x", Cam.pipeline.gx);
        telemetry.addData("y", Cam.pipeline.gy);
        telemetry.addData("gYELLOW", Cam.pipeline.greatestyellow);
        telemetry.update();

        int dx = Cam.pipeline.gx;
        int wthird = (int)(Math.floor(Cam.pipeline.w/3));

        while(runtime.seconds() < 5) {

        }

        /*
        if(dx <= wthird) {
            while(distance < 400) {
                d.drive(-0.4,0,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","LEFT");
                telemetry.update();
            }
        }

        else if(dx <= 2*wthird) {
            while(distance < 400) {
                d.drive(0,0.4,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","MIDDLE");
                telemetry.update();
            }
        }

        else {
            while(distance < 400) {
                d.drive(0.4,0,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","RIGHT");
                telemetry.update();
            }
        }*/
        /*
        while(distance < 1184) {
            d.drive(0,-0.4,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("distancee",o.distance);
            telemetry.update();
        }
        o.resetdistance(); distance=o.distance;
        while(distance < 1900) {
            d.drive(-0.4,0,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("dostance",o.distance);
            telemetry.update();
        }*/
    }
}
