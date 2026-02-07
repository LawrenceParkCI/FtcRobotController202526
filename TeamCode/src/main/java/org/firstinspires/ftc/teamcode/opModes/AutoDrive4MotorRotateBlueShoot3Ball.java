package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

@Autonomous(name="AutoDrive4MotorRotateBlueShoot3Ball", group="Autonomous")
public class AutoDrive4MotorRotateBlueShoot3Ball extends LinearOpMode {
    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Mechanism motors
    private DcMotor intake;
    private DcMotorEx shooter, carousel;
    // Smart servo
    private Servo pusher;
    private NormalizedColorSensor colorSensor;
    // Shooter RPM tracking
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;
    // Encoder specs (from manufacturer data)
    // Shooter 5202 motor (1:1) -> 28 pulses per motor revolution at output shaft.
    private static final double SHOOTER_PPR = 28.0;
    // Servo angle mapping if servo range is 300 degrees (±150) in standard mode.
    // Map 0..300 degrees -> 0.0..1.0 (adjust if your servo API expects different)
    private static final double SERVO_FULL_RANGE_DEG = 300.0;
    //Carousel rotation
    private static final double one3rd = 2786.2/3;
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
        driveForwardFixedTimeandStop(1.4, 1);
        rotateFixedTime(0.65, 1);
        long currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 200){
            mainDo();
        }
        rotateFixedTime(0.65, -1);
        driveForwardFixedTimeandStop(0.5, -1);
        for(idx = 0; idx <= 2; idx++){
            shoot(4000);
        }
        rotateFixedTime(0.95, 1);
        driveForwardFixedTimeandStop(2.0, 1);

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
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
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
        // Shooter and carousel: set mode
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Initialize pusher servo to 0 degrees (calibrated start)
        setServoAngle(pusher, 0.0);
        // Initialize carousel angle variable to 0 and ensure stopped
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Shooter motor reference: goBILDA 5202 1:1, 6000 rpm
        // Carousel motor reference: goBILDA 5202 99.5:1, ~60 rpm
        // Servo reference: Studica Multi-Mode Smart Servo (Standard Mode)
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
    private void setServoAngle(Servo s, double angleDeg) {
        // Map 0..SERVO_FULL_RANGE_DEG to 0..1 position
        double pos = RangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }
    private double RangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    //Shoots at target rpm
    private void shoot(double RPM){
        filter();
        setShooterTargetRPM(RPM);
        long currMilli = System.currentTimeMillis();
        while (opModeIsActive() && !targetMet && (System.currentTimeMillis() - currMilli < 8000)) {
            mainDo();
        }
        setServoAngle(pusher, 80.0);
        currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 200){
            mainDo();
        }
        setServoAngle(pusher, 0.0);
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 400){
            mainDo();
        }
        setShooterTargetRPM(0);
        mainDo();
    }
    private void filter(){
        char target = currentPattern[idx];
        rotateCarouselUntilColor(target);
    }
    private void rotateCarouselUntilColor(char color){
        int rotcount = 0;
        while(true){
            if(color == 'g' && isColorGreen()){
                break;
            }
            if(color == 'p' && isColorPurple()){
                break;
            }
            if(rotcount >= 6){
                break;
            }
            rotcount++;
            rotateThirdLeft();
        }
    }
    private void setShooterTargetRPM(double desiredRPM) {
        targetRPM = desiredRPM; // target minimum as specified
        // Compute ticks per second for desired rpm (we set motor velocity to achieve the desired RPM)
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        // DcMotorEx allows setting velocity in ticks per second
        shooter.setVelocity(ticksPerSec);
        // Immediately after changing speed, update actual currentRPM from encoder
        updateShooterRPM();
        targetMet = (currentRPM >= targetRPM);
    }
    private void updateShooterRPM() {
        //get ticks per second
        double ticksPerSec = shooter.getVelocity();
        //update current RPM ticksPerSec*RPM -> RPM
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
        targetMet = (currentRPM >= targetRPM);
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
        random();
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
    private void rotateThirdLeft(){
        long currPos = carousel.getCurrentPosition();
        carousel.setPower(-1);
        while(true ){
            mainDo();
            if(Math.abs(currPos  - carousel.getCurrentPosition()) >= one3rd){
                carousel.setPower(0);
                break;
            }
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
    private void random(){
        Random rand = new Random();
        int id = rand.nextInt(3);
        if (id == 0) {
            currentPattern = new char[]{'g', 'p', 'p'};
        } else if (id == 1) {
            currentPattern = new char[]{'p', 'g', 'p'};
        } else if (id == 2) {
            currentPattern = new char[]{'p', 'p', 'g'};
        }
    }
    private void mainDo(){
        updateAprilTagData();
        updateShooterRPM();
        updateTelemetry();
    }
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Pattern", currentPattern);
        telemetry.update();
    }
}
