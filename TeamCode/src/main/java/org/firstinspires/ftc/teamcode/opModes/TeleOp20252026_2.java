package org.firstinspires.ftc.teamcode.opModes;


import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.*;

@TeleOp(name = "TeleOp20252026_2", group = "TeleOp")
public class TeleOp20252026_2 extends LinearOpMode {
    // Drive motors (Control Hub)
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    // Expansion Hub motors
    private Carousel carousel;      // GoBilda 60 RPM gearbox (99.5:1) - encoder used for angle control
    private DcMotor intake;          // Tetrix
    private Shooter shooter;       // GoBilda 6000 RPM motor (1:1) with encoder

    //Gyro for field oriented driving.
    private IMU imu;
    private boolean fieldDriveMode = true;
    private double startHeading;

    //Intake State
    private boolean intakeActive = false;
    private long currTimeIntake=0;

    //Shooter state
    private long currTimeShooter= 0;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.clear();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        waitForStart();
        // Main loop
        while (opModeIsActive()) {
            drive();
            // --- INTAKE CONTROL on shooter (gamepad1) ---
            if(gamepad1.y && !intakeActive){
                shooter.start(-1200);
                // DcMotorEx allows setting velocity in ticks per second
                intakeActive = true;
                currTimeIntake = System.currentTimeMillis();
            }
            doAll();
            //timeout for shooter intake
            if(intakeActive && System.currentTimeMillis() - currTimeIntake >= 1750){
                intakeActive = false;
                shooter.stop();
            }
            doAll();

            //The real intake
            double intakePower = 0.0;
            if (gamepad1.left_bumper) {
                intakePower = -1.0; // backwards full power
            } else if (gamepad1.left_trigger > 0.05) {
                intakePower = gamepad1.left_trigger; // forward with analog speed from trigger
            } else {
                intakePower = 0.0;
            }
            // A limits max to 30%
            if (gamepad1.a) intakePower *= 0.30;
            intake.setPower(intakePower);

            if (gamepad1.backWasReleased())
                fieldDriveMode = !fieldDriveMode;


            // --- SHOOTER CONTROL with servo (gamepad2) ---
            // Buttons: X=1000 RPM, A=2500 RPM, B=4000 RPM, shoot and stop motor once done
            // When pressed, set motor velocity and update currentRPM and targetRPM logic.

            // We'll set shooter velocity in ticks per second, using SHOOTER_PPR pulses per rev:
            // desiredRPM -> ticksPerSec = desiredRPM * (SHOOTER_PPR / 60)

            doAll();
            getShooterControls();

            doAll();
            if(shooter.isShooterActive()){
                doAll();
                if(shooter.isTargetMet() && !shooter.isPusherActive()){
                    shooter.push();
                    currTimeShooter = System.currentTimeMillis();
                }

            }
            doAll();

            //stop the shooter 00 ms after servo pushed
            if(shooter.isPusherActive() && System.currentTimeMillis() - currTimeShooter >= 200){
                shooter.stop();
            }
            doAll();
            // --- CAROUSEL CONTROL (gamepad2) ---

            // Commands require waiting until the rotation is complete before processing next.
            //checks input from controller
            if(!carousel.isRotateActive()) {
                manualCarouselControls();
                autoCarouselControls();
            }

            // Check if move is complete or left_bumper pressed to cancel move
            if (carousel.isFinished() || carousel.isRotateActive() && gamepad2.back) {
                carousel.stop();
            }

//            carouselAngleDeg += rotateDegree;
//            carouselAngleDeg = normalizeAngle(carouselAngleDeg);

            doAll();
        }
    }

    private void autoCarouselControls(){
        //DPad Right/Left -> ±120°
        if (gamepad2.dpad_right) {
            carousel.rotateThirdRight();
        }
        if (gamepad2.dpad_left) {
            carousel.rotateThirdLeft();
        }
        if(gamepad2.dpad_up) {
            carousel.rotateSixthRight();
        }
        if(gamepad2.dpad_down){
            carousel.rotateSixthLeft();
        }
    }
    private void manualCarouselControls() {
        double carouselPower = 0.0;
        if (gamepad2.right_trigger > 0.05) {
            carouselPower = -gamepad2.right_trigger/2; // backwards
        } else if (gamepad2.left_trigger > 0.05) {
            carouselPower = gamepad2.left_trigger/2; // forward
        } else {
            carouselPower = 0.0;
        }
        carousel.move(carouselPower);
    }
    private void getShooterControls(){
        if(!shooter.isShooterActive()) {
            if (gamepad2.x) {
                shooter.start(1000);
            }
            if (gamepad2.a) {
                shooter.start(2500);
            }
            if (gamepad2.b) {
                shooter.start(4500);
            }
            if (gamepad2.y) {
                shooter.stop();
            }
        }
    }

    private void initHardware(){
        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        //retrieving imu from teleop???
        imu = MyGyro.imu;
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP
//        )));

        //creates the carousel starting in manual mode
        carousel = new Carousel(hardwareMap, Carousel.MANUAL);

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);

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

    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Current Shooter RPM:", shooter.getCurrentRPM());
        telemetry.addData("Current Target RPM:", String.format("%.1f", shooter.getTargetRPM()));
        telemetry.addData("Current Amperage ", shooter.getMotor().getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Target RPM Met?: ", shooter.isTargetMet());
        telemetry.addData("Carousel Positon", carousel.getMotor().getCurrentPosition());
        telemetry.addData("Carousel Power", carousel.getMotor().getPower());
        telemetry.addData("Carousel Active", carousel.isRotateActive());
        telemetry.addData("Last Position", carousel.getPosition());
        telemetry.addData("Carousel Delta", carousel.getMotor().getCurrentPosition() - carousel.getPosition());
        telemetry.addData("Carousel AMP", carousel.getMotor().getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    // --- Helper methods ---

    private void drive(){
        // --- DRIVE CONTROL (gamepad1) ---
        double lx = gamepad1.left_stick_x;   // left-stick left/right: strafing
        double ly = -gamepad1.left_stick_y;   // left-stick up/down: forward/back
        double rx = -gamepad1.right_stick_x;  // right-stick left/right: rotation
        if(fieldDriveMode) {
            if (gamepad1.bWasPressed()) {
                imu.resetYaw();
            }
            // Compute base motion powers
            // Mecanum drive mixing: forward/back = ly, strafe = lx, rotate = rx
            double theta = Math.atan2(ly, lx);
            double r = Math.hypot(lx, ly);

            theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            ly = r * Math.sin(theta);
            lx = r * Math.cos(theta);
        }

        // Compute base motion powers
        // Mecanum drive mixing: forward/back = ly, strafe = lx, rotate = rx
        double lf = ly + lx + rx;
        double rf = ly - lx - rx;
        double lb = ly - lx + rx;
        double rb = ly + lx - rx;

        // Normalize
        lf = rangeClip(-lf, -1.0,1.0);
        rf = rangeClip(-rf, -1.0,1.0);
        lb = rangeClip(-lb, -1.0,1.0);
        rb = rangeClip(-rb, -1.0,1.0);

        // Low speed mode: RT on gamepad1 limits to 30%
        double speedLimit = gamepad1.right_trigger > 0.05 ? 0.30 : 1.0;
        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }

    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void doAll(){
        shooter.updateRPM();
        updateTelemetry();
    }
}