package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 80, Math.toRadians(120), Math.toRadians(120), 15)
                .setDimensions(18, 18)
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60.00, Math.toRadians(90.00)))
//                        .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
//                        .waitSeconds(1)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(54, -36), Math.toRadians(0))
//                                .waitSeconds(1)
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
//                                .waitSeconds(1)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(54, -36), Math.toRadians(0))
//                                .waitSeconds(1)
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
//                                .waitSeconds(1)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(54, -36), Math.toRadians(0))
//                        .setReversed(false)
//                        .build()
//                        drive.trajectorySequenceBuilder(new Pose2d(-40.17, -68.00, Math.toRadians(99.46)))
//                        .splineTo(new Vector2d(-37.50, 27.50), Math.toRadians(88.40))
//                        .splineTo(new Vector2d(-13.17, 36.17), Math.toRadians(19.60))
//                        .splineTo(new Vector2d(18.00, 35.50), Math.toRadians(-1.23))
//                        .splineTo(new Vector2d(31.83, 28.67), Math.toRadians(260.54))
//                        .splineTo(new Vector2d(34.50, -1.17), Math.toRadians(-80.79))
//                        .splineTo(new Vector2d(36.00, -41.00), Math.toRadians(266.42))
//                        .splineTo(new Vector2d(35.50, -68.50), Math.toRadians(268.96))
//                        .splineTo(new Vector2d(25.67, -15.33), Math.toRadians(100.48))
//                        .splineTo(new Vector2d(-36.17, -12.17), Math.toRadians(177.07))
//                        .splineTo(new Vector2d(34.50, -5.33), Math.toRadians(5.52))
//                        .splineTo(new Vector2d(35.17, 29.33), Math.toRadians(54.21))
//                        .splineTo(new Vector2d(62.67, 27.50), Math.toRadians(-3.81))
//                        .splineTo(new Vector2d(68.17, -20.67), Math.toRadians(-83.49))
//                        .splineTo(new Vector2d(33.00, -26.33), Math.toRadians(189.15))
//                        .build()
//                                drive.trajectorySequenceBuilder(new Pose2d(-61.03, -34.97, Math.toRadians(0.00)))
//                                        .splineToConstantHeading(new Vector2d(-46.55, -14), Math.toRadians(0))
//                                        .splineToConstantHeading(new Vector2d(44, -14), Math.toRadians(0))
////                                        .splineToConstantHeading(new Vector2d(56, -33), Math.toRadians(0))
//                                        .splineToConstantHeading(new Vector2d(56, 60), Math.toRadians(0))
//                                        .build()
                                drive.trajectorySequenceBuilder(new Pose2d(-57.12, -12.00, Math.toRadians(0.00)))
                                        .splineTo(new Vector2d(7.00, -12.00), Math.toRadians(0.00))
                                        .splineToLinearHeading(new Pose2d(55.61, -28.00, Math.toRadians(0.00)), Math.toRadians(-18.55))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(6.87, -14.49), Math.toRadians(173.86))
                                        .splineTo(new Vector2d(-56.97, -13.48), Math.toRadians(180.00))
                                        .setReversed(false)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}