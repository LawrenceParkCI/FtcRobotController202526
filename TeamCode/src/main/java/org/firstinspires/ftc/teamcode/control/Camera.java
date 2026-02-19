package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Camera {


    int cameraPositionTolerance = 5;
    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    // Data structures
    private final Map<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();
    private List<AprilTagDetection> detections;

    public Camera(HardwareMap hardwareMap){
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        aprilTag = tagBuilder.build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
        visionPortal.resumeStreaming();

        detections = new ArrayList<AprilTagDetection>();
    }
    public boolean isFacingTag(int reqID){
        for (AprilTagDetection det : detections) {
            double position = det.ftcPose.x;
            if(det.id == reqID && position > -cameraPositionTolerance && position < cameraPositionTolerance) {
                return true;
            }
        }
        return false;
    }

    public double getFacing(int reqID){
        for (AprilTagDetection det : detections) {
            if(det.id == reqID)
                return det.ftcPose.x;
        }
        return -1;
    }

    /**
     * Gives the distance to the required ID tag. Returns -1 if the
     * specified April Tag is not found.
     * @param reqID the ID we are looking for
     * @return the distance from the tag, or -1 if not found
     */
    public double getDistance(int reqID){
        for (AprilTagDetection det : detections) {
            if(det.id == reqID)
                return det.ftcPose.range;
        }
            return -1;
    }

    /**
     * Finds the pattern on the obelisk.
     * @return a char array with the pattern found
     */
    public char[]  getPattern() {
        int id =0;
        for (AprilTagDetection det : detections) {
            id = det.id;

            if (id == 21) {
                return new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                return new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                return new char[]{'p', 'p', 'g'};
            }
            // ID 20 = blue alliance scoring, ID 24 = red alliance scoring
        }
        return new char[]{'a', 'a', (char)(id%10 + 97)};
    }

    public void  updateAprilTagData() {
        detections = aprilTag.getDetections();
        seenTagIds.clear();
        idToDistanceMeters.clear();
        for (AprilTagDetection det : detections) {
            int id = det.id;
            double distanceMeters = det.ftcPose.range;
            idToDistanceMeters.put(id, distanceMeters);
            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }
        }

    }

    public List<Integer> getSeenTagIds(){
        return seenTagIds;
    }

    public Map<Integer, Double> getIdsToDistance(){
        return idToDistanceMeters;
    }

    public void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }


    public void display(Telemetry telemetry){
        for (AprilTagDetection det : detections) {
            telemetry.addData("ID: ", det.id);
        }
    }
}
