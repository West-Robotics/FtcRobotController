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
                .setConstraints(60, 140, Math.toRadians(120), Math.toRadians(120), 15)
                .setDimensions(18, 18)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-36,-62,Math.toRadians(-90)))
                    .setReversed(true)
                    // GOTO GROUND PIXEL
                    .splineToConstantHeading(new Vector2d(-47,-48), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-47,-21), Math.toRadians(90))
                    //.stopAndAdd(motorActions.claw.release())


                    // GOTO STACK
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-64,-12,Math.toRadians(180)), Math.toRadians(180))
                    //.stopAndAdd(motorActions.claw.grab())


                    // GOTO BACKBOARD
                    .setReversed(true)
                    //.afterTime(0.5, motorActions.pixelToHook())
                    .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                    //.afterTime(0.5, drive.CorrectWithTagAction())
                    .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                    //.stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))

                    // GOTO STACK
                    .setTangent(Math.toRadians(180))
                    .setReversed(false)
                    .splineTo(new Vector2d(20, -60), Math.toRadians(180))
                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-64,-36,Math.toRadians(180)), Math.toRadians(180))
                    //.stopAndAdd(motorActions.claw.grab())


                    // GOTO BACKBOARD
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                    //.afterTime(0.5, drive.CorrectWithTagAction())
                    .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                    //.stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))

                    // GOTO STACK
                    .setTangent(Math.toRadians(180))
                    .setReversed(false)
                    .splineTo(new Vector2d(20, -60), Math.toRadians(180))
                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-64,-36,Math.toRadians(180)), Math.toRadians(180))
                    //.stopAndAdd(motorActions.claw.grab())




                    // GOTO BACKBOARD
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                    //.afterTime(0.5, drive.CorrectWithTagAction())
                    .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                    //.stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))

                    .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}