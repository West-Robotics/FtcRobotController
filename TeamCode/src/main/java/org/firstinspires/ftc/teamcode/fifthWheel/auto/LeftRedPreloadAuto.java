// package org.firstinspires.ftc.teamcode.fifthWheel.auto;
// 
// import com.acmerobotics.dashboard.FtcDashboard;
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// 
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
// import org.firstinspires.ftc.teamcode.drive.DriveConstants;
// import org.firstinspires.ftc.teamcode.Controller;
// import org.firstinspires.ftc.teamcode.fifthWheel.command.Place;
// import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
// import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.IntakeCam;
// import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
// 
// import java.lang.Math;
// 
// @Autonomous(name="Left Red Preload", group="FifthWheel")
// public class LeftRedPreloadAuto extends LinearOpMode {
// 
//     enum State {
//         DROPPING_PRELOAD,
//         CONE_STACK,
//         LAST_CONE_STACK,
//         PARKING,
//         IDLING
//     } State state = State.IDLING;
// 
//     @Override
//     public void runOpMode() throws InterruptedException {
//         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//         Place place = new Place(hardwareMap, "liftLeft", "liftRight", "touch", "flipLeft", "flipRight", "grip", Gripper.Alliance.RED);
//         IntakeCam inCam = new IntakeCam(hardwareMap, true);
//         Controller controller = new Controller(gamepad1);
//         int mode = 0;
//         int signal = 1;
//         int coneCount = 5;
// 
//         double halfLength = 11.75/2.0;
// 
//         // TODO: adjust for new dt plates
//         Pose2d startPose = new Pose2d(-36, -72+halfLength+1.875, Math.toRadians(90));
//         Pose2d lineupPose = new Pose2d(-72+22.75+halfLength+1.875, -11.5, Math.toRadians(180));
// 
//         drive.setPoseEstimate(startPose);
// 
//         TrajectorySequence preloadDrop = drive.trajectorySequenceBuilder(startPose)
//             .UNSTABLE_addTemporalMarkerOffset(0, () -> place.pickup())
//             .waitSeconds(0.3)
//             .splineTo(new Vector2d(-36, -72+30), Math.toRadians(90),
//                 SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(200), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(40))
//             .UNSTABLE_addTemporalMarkerOffset(-1, () -> place.raise(3))
//             .splineTo(new Vector2d(-31, -72+65), Math.toRadians(45),
//                 SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(200), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(40))
//             .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> place.dip(true))
//             .UNSTABLE_addTemporalMarkerOffset(0, () -> place.dropAndLower())
//             .UNSTABLE_addTemporalMarkerOffset(0.0, () -> inCam.beginConeStack())
//             .lineToSplineHeading(lineupPose,
//                 SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(200), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(40))
//             .build();
// 
//         TrajectorySequence coneStack = drive.trajectorySequenceBuilder(preloadDrop.end())
//             .UNSTABLE_addTemporalMarkerOffset(0.1,
//                     () -> drive.setPoseEstimate(
//                                     new Pose2d(-72.0+inCam.getYDistance()-2.25,
//                                                -11.5+1.2-inCam.getXDistance(drive.getPoseEstimate().getHeading()),
//                                                 drive.getPoseEstimate().getHeading())))
//             .UNSTABLE_addTemporalMarkerOffset(0.11,
//                     () -> updateCones(inCam, drive))
//             .waitSeconds(0.11)
//             .UNSTABLE_addTemporalMarkerOffset(0.4, () -> place.goToStack())
//             .lineToSplineHeading(new Pose2d(-65.85, -11.5, Math.toRadians(180)),
//                 SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(20))
//             .UNSTABLE_addTemporalMarkerOffset(0, () -> place.liftOffStack())
//             .UNSTABLE_addTemporalMarkerOffset(0.7, () -> place.raise(3))
//             .lineToSplineHeading(new Pose2d(32, -72+64, Math.toRadians(45)),
//                 SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(30))
//             .UNSTABLE_addTemporalMarkerOffset(0.1, () -> place.dip(true))
//             .waitSeconds(0.2)
//             .UNSTABLE_addTemporalMarkerOffset(0.0, () -> place.dropAndLower())
//             .lineToSplineHeading(lineupPose,
//                 SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(30))
//             .build();
// 
//         TrajectorySequence lastConeStack = drive.trajectorySequenceBuilder(coneStack.end())
//             .UNSTABLE_addTemporalMarkerOffset(0.5,
//                     () -> drive.setPoseEstimate(
//                                     new Pose2d(72.0-inCam.getYDistance()-halfLength-1.875,
//                                                -11.5-4.75+inCam.getXDistance(drive.getPoseEstimate().getHeading()),
//                                                 drive.getPoseEstimate().getHeading())))
//             .UNSTABLE_addTemporalMarkerOffset(0.55,
//                     () -> updateCones(inCam, drive))
//             .waitSeconds(0.75)
//             .lineToSplineHeading(new Pose2d(59, -11.5, Math.toRadians(0)),
//                 SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(20))
//             .waitSeconds(0.5)
//             .lineToSplineHeading(new Pose2d(33, -72+62, Math.toRadians(135)),
//                 SampleMecanumDrive.getVelocityConstraint(38, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//                 SampleMecanumDrive.getAccelerationConstraint(38))
//             .build();
// 
//         // TrajectorySequence coneStack = drive.trajectorySequenceBuilder(preloadDrop.end())
//         //     .UNSTABLE_addTemporalMarkerOffset(0.5,
//         //             () -> drive.setPoseEstimate(
//         //                         new Pose2d(72.0-inCam.getYDistance()-halfLength-1.875,
//         //                                    -11.5-4.5+inCam.getXDistance(drive.getPoseEstimate().getHeading()),
//         //                                    drive.getPoseEstimate().getHeading())))
//         //     .UNSTABLE_addTemporalMarkerOffset(0.55,
//         //             () -> updateCones(inCam, drive))
//         //     .setTurnConstraint(Math.toRadians(200), Math.toRadians(200))
//         //     .waitSeconds(0.75)
//         //     .lineToSplineHeading(new Pose2d(59, -11.5, Math.toRadians(0)),
//         //         SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
//         //         SampleMecanumDrive.getAccelerationConstraint(20))
//         //     .waitSeconds(0.5)
//         //     .back(35,
//         //         SampleMecanumDrive.getVelocityConstraint(38, Math.toRadians(200), DriveConstants.TRACK_WIDTH),
//         //         SampleMecanumDrive.getAccelerationConstraint(38))
//         //     .turn(Math.toRadians(90))
//         //     .waitSeconds(0.15)
//         //     .turn(Math.toRadians(-90))
//         //     .lineToSplineHeading(lineupPose,
//         //         SampleMecanumDrive.getVelocityConstraint(38, Math.toRadians(200), DriveConstants.TRACK_WIDTH),
//         //         SampleMecanumDrive.getAccelerationConstraint(38))
//         //     .build();
// 
//         TrajectorySequence Park = null;
// 
//         TrajectorySequence signalOne = drive.trajectorySequenceBuilder(coneStack.end())
//             .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90)))
//             .strafeTo(new Vector2d(-60, -36))
//             .build();
// 
//         // TrajectorySequence signalOne = drive.trajectorySequenceBuilder(coneStack.end())
//         //     .splineToLinearHeading(new Pose2d(36, -38, Math.toRadians(90)), Math.toRadians(90))
//         //     .splineToLinearHeading(new Pose2d(12, -38, Math.toRadians(90)), Math.toRadians(90))
//         //     .build();
// 
//         TrajectorySequence signalTwo = drive.trajectorySequenceBuilder(coneStack.end())
//             .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90)))
//             .build();
// 
//         // TrajectorySequence signalTwo = drive.trajectorySequenceBuilder(coneStack.end())
//         //     .splineToLinearHeading(new Pose2d(36, -38, Math.toRadians(90)), Math.toRadians(90))
//         //     .build();
// 
//         TrajectorySequence signalThree = drive.trajectorySequenceBuilder(coneStack.end())
//             .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90)))
//             .strafeTo(new Vector2d(-12, -36))
//             .build();
// 
//         // TrajectorySequence signalThree = drive.trajectorySequenceBuilder(coneStack.end())
//         //     .splineToLinearHeading(new Pose2d(36, -38, Math.toRadians(90)), Math.toRadians(90))
//         //     .splineToLinearHeading(new Pose2d(60, -38, Math.toRadians(90)), Math.toRadians(90))
//         //     .build();
// 
//         FtcDashboard.getInstance().startCameraStream(IntakeCam.camera, 0);
// 
//         while (!isStarted() && !isStopRequested()) {
//             place.intake();
//             controller.update();
//             if (controller.A()) {
//                 mode = 0;
//             } else if (controller.B()) {
//                 mode = 1;
//             }
//             if (mode == 0) {
//                 if (controller.dpadUp()) {
//                     inCam.changeCornerY(5);
//                 }
//                 if (controller.dpadDown()) {
//                     inCam.changeCornerY(-5);
//                 }
//                 if (controller.dpadLeft()) {
//                     inCam.changeCornerX(-5);
//                 }
//                 if (controller.dpadRight()) {
//                     inCam.changeCornerX(5);
//                 }
//             }
//             if (mode == 1) {
//                 if (controller.dpadUp()) {
//                     inCam.changeRegionHeight(-5);
//                 }
//                 if (controller.dpadDown()) {
//                     inCam.changeRegionHeight(5);
//                 }
//                 if (controller.dpadLeft()) {
//                     inCam.changeRegionWidth(-5);
//                 }
//                 if (controller.dpadRight()) {
//                     inCam.changeRegionWidth(5);
//                 }
//             }
//             telemetry.addData("regionWidth", inCam.getRegionWidth());
//             telemetry.addData("regionHeight", inCam.getRegionHeight());
//             telemetry.addData("cornerX", inCam.getCornerX());
//             telemetry.addData("cornerY", inCam.getCornerY());
//             telemetry.addData("Signal face", inCam.getSignalFace());
//             telemetry.addData("Avg hue", inCam.getAvgHue());
//             telemetry.addData("pixelWidth", inCam.getPixelWidth());
//             telemetry.addData("midWidth", inCam.getMidWidth());
//             telemetry.addData("y distance", inCam.getYDistance());
//             telemetry.addData("x distance", inCam.getXDistance(drive.getPoseEstimate().getHeading()));
//             telemetry.update();
// 
//             // Don't burn CPU cycles busy-looping in this sample
//             sleep(200);
//         }
// 
//         signal = inCam.getSignalFace();
// 
//         switch (signal) {
//             case 1:
//                 Park = signalOne;
//                 break;
//             case 2:
//                 Park = signalTwo;
//                 break;
//             case 3:
//                 Park = signalThree;
//                 break;
//         }
// 
//         state = State.DROPPING_PRELOAD;
//         drive.followTrajectorySequenceAsync(preloadDrop);
//         while (opModeIsActive() && !isStopRequested()) {
//             switch (state) {
//                 case DROPPING_PRELOAD:
//                     if (!drive.isBusy()) {
//                         state = State.PARKING;
//                         drive.followTrajectorySequenceAsync(Park);
//                     }
//                     break;
//                 case CONE_STACK:
//                     if (!drive.isBusy() && coneCount > 4) {
//                         coneCount--;
//                         drive.followTrajectorySequenceAsync(coneStack);
//                     } else if (!drive.isBusy() && coneCount == 4) {
// //                        state = State.LAST_CONE_STACK;
// //                        drive.followTrajectorySequenceAsync(lastConeStack);
//                         state = State.PARKING;
//                         drive.followTrajectorySequenceAsync(Park);
//                     }
//                     break;
//                 case LAST_CONE_STACK:
//                     if (!drive.isBusy()) {
//                         state = State.PARKING;
//                         drive.followTrajectorySequenceAsync(Park);
//                     }
//                     break;
//                 case PARKING:
//                     if (!drive.isBusy()) {
//                         state = State.IDLING;
//                     }
//                     break;
//                 case IDLING:
//                     break;
//             }
//             drive.update();
//             place.run(0);
//         }
//     }
//     void updateCones(IntakeCam cam, SampleMecanumDrive d) {
//         telemetry.addData("pixelWidth", cam.getPixelWidth());
//         telemetry.addData("midWidth", cam.getMidWidth());
//         telemetry.addData("y distance", cam.getYDistance());
//         telemetry.addData("x distance", cam.getXDistance(d.getPoseEstimate().getHeading()));
//         telemetry.update();
//     }
// }
