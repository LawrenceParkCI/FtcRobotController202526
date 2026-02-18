package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.Carousel;
import org.firstinspires.ftc.teamcode.control.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Autonomous(name="AutoDrive4MotorRotateRedShoot3Ball", group="Autonomous")
public class AutoDrive4MotorRotateRedShoot3Ball extends LinearOpMode {
    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Mechanism motors
    private DcMotor intake;
    private Shooter shooter;
    private Carousel carousel;
    private NormalizedColorSensor colorSensor;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    // Data structures
    private final Map<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();
    private char[] currentPattern = new char[3];
    int idx = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initVision();
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        if (isStopRequested()) {
            shutdownVision();
            return;
        }
        waitForStart();
        telemetry.clearAll();
        telemetry.update();
        driveForwardFixedTimeandStop(0.7, 1);
        rotateFixedTime(0.8, -0.8);
        long currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 200){
            mainDo();
        }
        rotateFixedTime(0.7, 0.8);
//        driveForwardFixedTimeandStop(0.5, -1);
//        for(idx = 0; idx <= 2; idx++){
//            shoot(2500);
//        }
        hardCodeShoot(2500);
        rotateFixedTime(0.5, -1);
        driveForwardFixedTimeandStop(1.4, 1);

        // Standstill, keep updating AprilTag data
        while (opModeIsActive()) {
            if (isStopRequested()) {
                shutdownVision();
                return;
            }
        }
        shutdownVision();
    }
    private void initHardware() {
        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        carousel = new Carousel(hardwareMap, Carousel.AUTO);
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");
        // Set drive motor directions (adjust if your robot's wiring is different)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // Brake when power is zero for precise stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake motor direction default
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Carousel motor reference: goBILDA 5202 99.5:1, ~60 rpm
        // Alliance tags: ID 20 (blue scoring), ID 24 (red scoring)
    }
    private void driveForwardFixedTimeandStop(double seconds, double power) {
        setDrivePower(power);
        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            mainDo();
        }
        stopDrive();
    }
    private void setDrivePower(double p) {
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }
    //CCW - negative
    //CW - positive;
    private void rotateFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setRotatePower(power);
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
           mainDo();
        }
        stopDrive();
    }
    private void setRotatePower(double p){
        leftFront.setPower(p);
        rightFront.setPower(-p);
        leftBack.setPower(p);
        rightBack.setPower(-p);
    }
    private void stopDrive() {
        setDrivePower(0.0);
    }

    /**
     * Assumes green ball is preset in center spot
     * @param RPM
     */
    private void hardCodeShoot(double RPM){

        if(currentPattern[0] == 'g'){
            for(int i = 0; i < 2; i++) {
                shoot(RPM);
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()){
                    //TODO ensure still facing AprilTag on Goal
                    mainDo();
                }
            }
            shoot(RPM);
        }
        else if (currentPattern[1] == 'g') {
            carousel.rotateThirdRight();
            while (!carousel.isFinished()) ;
            for(int i = 0; i < 2; i++) {
                shoot(RPM);
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()) {
                    //TODO ensure still facing AprilTag on Goal
                    mainDo();
                }
            }
            shoot(RPM);
        }
        else {
            for(int i = 0; i < 3; i++) {
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()) {
                    //TODO ensure still facing AprilTag on Goal
                    mainDo();
                }
                shoot(RPM);
            }
        }

    }

    private void shoot(double RPM){
        shooter.start(RPM);
        while(!shooter.isTargetMet()){
            //TODO ensure still facing AprilTag on Goal
            mainDo();
        }
        shooter.push();

        //pause 200 ms
        long currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 200){
            mainDo();
        }
        shooter.stop();
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
    private float[] readColor(){
        // Read normalized RGBA values
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        // Convert to HSV
        Color.colorToHSV(colors.toColor(), hsv);

        return hsv;
    }
    private boolean isColorGreen(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 120 && hue <= 180);
    }
    private boolean isColorPurple(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 181 && hue <= 245);
    }

    //Don't need this??
//    private void random(){
//        Random rand = new Random();
//        int id = rand.nextInt(3);
//        if (id == 0) {
//            currentPattern = new char[]{'g', 'p', 'p'};
//        } else if (id == 1) {
//            currentPattern = new char[]{'p', 'g', 'p'};
//        } else if (id == 2) {
//            currentPattern = new char[]{'p', 'p', 'g'};
//        }
//    }
    private void mainDo(){
        updateAprilTagData();
        shooter.updateRPM();
        updateTelemetry();
    }
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Pattern", Arrays.toString(currentPattern));
        telemetry.update();
    }

}
