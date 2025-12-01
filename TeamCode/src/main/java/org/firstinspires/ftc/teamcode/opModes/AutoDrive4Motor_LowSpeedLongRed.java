package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="AutoDrive4Motor_LowSpeedLongRed", group="Autonomous")
public class AutoDrive4Motor_LowSpeedLongRed extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Vision
    /*private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Data structures

    private final HashMap<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();
    private char[] currentPattern = null;
    */

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.clear();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        //initVision();

        waitForStart();
        telemetry.clearAll();
        telemetry.update();
        /*if (isStopRequested()) {
            //shutdownVision();
            return;
        }*/


        // Drive forward for 1.2s
        driveForwardFixedTime(1.2, -1);
        rotateFixedTime(0.575, 1);
        driveForwardFixedTime(1.2, -1);
        stopDrive();

        // Standstill, keep updating AprilTag data
        while (opModeIsActive()) {
            //updateAprilTagData();
        }
        //shutdownVision();
    }
    private void rotateThirdRight(){

        carousel.setVelocity(900);
        sleep(402);
        carousel.setPower(0);
        carouselAngleDeg += 120;
        carouselAngleDeg = normalizeAngle(carouselAngleDeg);
    }
    private void rotateThirdLeft(){
        carousel.setVelocity(-900);
        sleep(402);
        carousel.setPower(0);
        carouselAngleDeg -= 120;
        carouselAngleDeg = normalizeAngle(carouselAngleDeg);
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
        setShooterTargetRPM(RPM);
        updateShooterRPM();
        telemetry.clearAll();
        telemetry.addData("Carousel Degree: ", carouselAngleDeg);
        telemetry.addData("Current Shooter RPM:", String.format("%.1f", currentRPM));
        telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
        telemetry.addData("Current Amperage ", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("targetMet", targetMet);
        telemetry.update();
        long currMilli = System.currentTimeMillis();
        while (opModeIsActive() && !targetMet &&(System.currentTimeMillis() - currMilli < 8000)) {
            updateShooterRPM();
            telemetry.clearAll();
            telemetry.addData("Carousel Degree: ", carouselAngleDeg);
            telemetry.addData("Current Shooter RPM:", String.format("%.1f", currentRPM));
            telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
            telemetry.addData("Current Amperage ", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("targetMet", targetMet);
            telemetry.update();
        }
        sleep(500);
        setServoAngle(pusher, 80.0);
        sleep(200); // short wait to allow movement (adjust as needed)
        setServoAngle(pusher, 0.0);
        sleep(500);
        setShooterTargetRPM(0);
        updateShooterRPM();
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
    private double normalizeAngle(double a) {
        double v = a % 360.0;
        if (v < 0) v += 360.0;
        return v;
    }
    private void initHardware() {
        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

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
        // Alliance tags: ID 20 (blue scoring), ID 24 (red scoring)
    }
    private void driveForwardFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setDrivePower(power);
        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            //updateAprilTagData();
        }
        stopDrive();
    }
    private void setDrivePower(double p) {
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }
    private void setRotatePower(double p){
        leftFront.setPower(p);
        rightFront.setPower(-p);
        leftBack.setPower(p);
        rightBack.setPower(-p);
    }
    //CCW - negative
    //CW - positive;
    private void rotateFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setRotatePower(power);
        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            //updateAprilTagData();
        }
        stopDrive();
    }
    private void stopDrive() {
        setDrivePower(0.0);
    }
    /*
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
    }*/
}