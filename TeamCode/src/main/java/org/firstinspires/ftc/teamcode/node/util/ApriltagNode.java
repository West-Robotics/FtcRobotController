//package org.firstinspires.ftc.teamcode.node.util;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.node.Node;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.HashMap;
//import java.util.List;
//
//public class ApriltagNode extends Node {
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection tag = new AprilTagDetection();
//    private VisionPortal visionPortal;
//
//    public ApriltagNode(HardwareMap hardwareMap, String camName, double fx, double fy, double cx, double cy) {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(false)
//                .setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(fx, fy, cx, cy)
////                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                // set pose solver
//                .build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, camName));
//        builder.setCameraResolution(new Size(640, 480));
//        builder.enableCameraMonitoring(true);
//        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//        builder.setAutoStopLiveView(false);
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//    }
//    @Override
//    public void end() {
//        visionPortal.close();
//    }
//    public HashMap<String, Object> publish() {
//        HashMap<String, Object> message = new HashMap<String, Object>();
//        // WARNING: returns last known data if no tags are present
//        if (!aprilTag.getDetections().isEmpty()) {
//            tag = aprilTag.getDetections().get(0);
//        }
//        message.put("tag x", tag.ftcPose.x);
//        message.put("tag y", tag.ftcPose.y);
//        return message;
//    }
//}