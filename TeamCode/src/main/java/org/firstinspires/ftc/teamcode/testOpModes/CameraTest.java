package org.firstinspires.ftc.teamcode.testOpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Autonomous(name="CameraTest", group="Autonomous")
public class CameraTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private final Map<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();
    private char[] currentPattern = new char[3];
    public void runOpMode() throws InterruptedException {
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        initVision();
        if (isStopRequested()) {
            shutdownVision();
            return;
        }
        waitForStart();
        while (opModeIsActive()) {
            updateAprilTagData();
            telemetry.clearAll();
            telemetry.addData("Seen AprilTags", seenTagIds);
            telemetry.addData("Distance to AprilTags", idToDistanceMeters);
            telemetry.addData("Pattern", Arrays.toString(currentPattern));
            telemetry.update();
        }
        shutdownVision();

    }
    private void initVision() {
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        aprilTag = tagBuilder.build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
        visionPortal.resumeStreaming();
    }
    private void updateAprilTagData() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        seenTagIds.clear();
        idToDistanceMeters.clear();
        for (AprilTagDetection det : detections) {
            int id = det.id;
            double distanceMeters = det.ftcPose.range;
            idToDistanceMeters.put(id, distanceMeters);
            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }

            if (id == 21) {
                currentPattern = new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                currentPattern = new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                currentPattern = new char[]{'p', 'p', 'g'};
            }
            // ID 20 = blue alliance scoring, ID 24 = red alliance scoring
        }
    }

    private void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
}
