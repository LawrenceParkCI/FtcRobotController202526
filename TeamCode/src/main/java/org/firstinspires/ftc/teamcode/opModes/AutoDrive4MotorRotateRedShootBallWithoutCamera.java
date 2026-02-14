package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.Carousel;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
//Red
@Autonomous(name="AutoDrive4MotorRotateRedShootBallWithoutCamera", group="Autonomous")
public class AutoDrive4MotorRotateRedShootBallWithoutCamera extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Mechanism motors
    private DcMotor intake;
    private DcMotorEx shooter;
    Carousel carousel;
    // Smart servo
    private Servo pusher;

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
    private int current = 0;
    private static final double CAROUSEL_PPR = 2786.2;
    private static final double CAROUSEL_PPR3rd = CAROUSEL_PPR/3;
    


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        

        waitForStart();
        telemetry.clearAll();
        telemetry.update();
        if (isStopRequested()) {
            return;
        }

        // Drive forward for total 3.1s
        driveForwardFixedTime(0.7, 1);
        sleep(200);
        shoot(4000);
        carousel.rotateThirdLeft();
        while(!carousel.isFinished()){sleep(50);}
        shoot(4000);
        carousel.rotateThirdLeft();
        while(!carousel.isFinished()){sleep(50);}
        shoot(4000);
        sleep(200);
        rotateFixedTime(0.25, -1);
        sleep(200);
        driveForwardFixedTime(1.4, 1);
        stopDrive();

        // Standstill, keep updating AprilTag data
        while (opModeIsActive()) {
        }
    }

    private void initHardware() {
        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        carousel = new Carousel(Carousel.AUTO);
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");

        pusher = hardwareMap.get(Servo.class, "pusher");

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

        // Shooter motor reference: goBILDA 5202 1:1, 6000 rpm
        // Carousel motor reference: goBILDA 5202 99.5:1, ~60 rpm
        // Servo reference: Studica Multi-Mode Smart Servo (Standard Mode)
        // Alliance tags: ID 20 (blue scoring), ID 24 (red scoring)
    }
    private void driveForwardFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setDrivePower(power);
        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
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
        }
        stopDrive();
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
        setShooterTargetRPM(RPM);
        updateShooterRPM();
        long currMilli = System.currentTimeMillis();
        while (opModeIsActive() && !targetMet && (System.currentTimeMillis() - currMilli < 8000)) {
            updateShooterRPM();
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
}
