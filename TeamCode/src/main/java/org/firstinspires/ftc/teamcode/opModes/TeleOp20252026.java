package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends LinearOpMode {

    // Drive motors (Control Hub)
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Expansion Hub motors
    private DcMotorEx carousel;      // GoBilda 60 RPM gearbox (99.5:1) - encoder used for angle control
    private DcMotor intake;          // Tetrix
    private DcMotorEx shooter;       // GoBilda 6000 RPM motor (1:1) with encoder

    // Servo
    private Servo pusher;            // multimode smart servo (angular mode), initialized to 0 degrees

    // Shooter RPM tracking
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;


    // Encoder specs (from manufacturer data)
    // Shooter 5202 motor (1:1) -> 28 pulses per motor revolution at output shaft.
    private static final double SHOOTER_PPR = 28.0;

    // Carousel gearbox motor (99.5:1) -> ~2786.2 pulses per output shaft revolution.
    private static final double CAROUSEL_PPR = 2786.2;

    // Servo angle mapping if servo range is 300 degrees (±150) in standard mode.
    // Map 0..300 degrees -> 0.0..1.0 (adjust if your servo API expects different)
    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    // Carousel current angle in degrees [0,360)
    private double carouselAngleDeg = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
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

        // Initialize carousel angle variable to 0 and ensure stopped
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselAngleDeg = 0.0; // encoder is zeroed above


        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {

            // --- DRIVE CONTROL (gamepad1) ---
            double lx = -gamepad1.left_stick_x;   // left-stick left/right: strafing
            double ly = -gamepad1.left_stick_y;   // left-stick up/down: forward/back
            double rx =  gamepad1.right_stick_x;  // right-stick left/right: rotation

            // Compute base motion powers
            // Mecanum drive mixing: forward/back = ly, strafe = lx, rotate = rx
            double lf = ly + lx + rx;
            double rf = ly - lx - rx;
            double lb = ly - lx + rx;
            double rb = ly + lx - rx;

            // Normalize
            double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
            lf /= max; rf /= max; lb /= max; rb /= max;

            // Low speed mode: RT on gamepad1 limits to 30%
            double speedLimit = gamepad1.right_trigger > 0.05 ? 0.30 : 1.0;
            leftFront.setPower(lf * speedLimit);
            rightFront.setPower(rf * speedLimit);
            leftBack.setPower(lb * speedLimit);
            rightBack.setPower(rb * speedLimit);

            // --- INTAKE CONTROL (gamepad1) ---
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

            // --- SHOOTER CONTROL with servo (gamepad2) ---
            // Buttons: X=1000 RPM, A=2500 RPM, B=4000 RPM, shoot and stop motor once done
            // When pressed, set motor velocity and update currentRPM and targetRPM logic.

            // We'll set shooter velocity in ticks per second, using SHOOTER_PPR pulses per rev:
            // desiredRPM -> ticksPerSec = desiredRPM * (SHOOTER_PPR / 60)
            if (gamepad2.x) {
                shoot(1000);
            } else if (gamepad2.a) {
                shoot(2500);
            } else if (gamepad2.b) {
                shoot(4000);
            }
            updateShooterRPM();

           /* // --- PUSHER SERVO (gamepad2 RB) ---
            if (gamepad2.right_bumper) {
                // rotate clockwise 80 degrees then back immediately and set power to 0 (servo movement done)
                // We implement as a quick move; in a real robot you might want async or timed control.
                setServoAngle(pusher, 80.0);
                sleep(200); // short wait to allow movement (adjust as needed)
                setServoAngle(pusher, 0.0);
                sleep(100);
            }*/
            // --- Carousel CONTROL (gamepad2) ---
            double carouselPower = 0.0;
            if (gamepad2.right_trigger > 0.05) {
                carouselPower = gamepad2.right_trigger; // backwards full
            } else if (gamepad2.left_trigger > 0.05) {
                carouselPower = -gamepad2.left_trigger; // forwards
            } else {
                carouselPower = 0.0;
            }
            double carousel_lim = 50;
            carousel.setVelocity(carouselPower*carousel_lim);

            // --- CAROUSEL CONTROL (gamepad2) ---
            // Commands require waiting until the rotation is complete before processing next.
            // LB + DPad Right/Left -> ±60° ; DPad Right/Left -> ±120°
            if (gamepad2.dpad_up) {
                moveCarouselByDegrees(60.0);
            } else if (gamepad2.dpad_down) {
                moveCarouselByDegrees(-60.0);
            } else if (gamepad2.dpad_right) {
                moveCarouselByDegrees(120.0);
            } else if (gamepad2.dpad_left) {
                moveCarouselByDegrees(-120.0);
            }

            // --- TELEMETRY ---
            telemetry.clearAll();
            telemetry.addData("Current Shooter RPM:", String.format("%.1f", currentRPM));
            telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
            telemetry.addData("targetMet", targetMet);
            telemetry.update();

            idle();
        }
    }

    // --- Helper methods ---
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
        telemetry.addData("Current Shooter RPM:", String.format("%.1f", currentRPM));
        telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
        telemetry.addData("targetMet", targetMet);
        telemetry.update();
        while (opModeIsActive() && currentRPM < targetRPM) {
            updateShooterRPM();
            telemetry.clearAll();
            telemetry.addData("Current Shooter RPM:", String.format("%.1f", currentRPM));
            telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
            telemetry.addData("targetMet", targetMet);
            telemetry.update();
        }
        setServoAngle(pusher, 80.0);
        sleep(200); // short wait to allow movement (adjust as needed)
        setServoAngle(pusher, 0.0);
        sleep(1000);
        setShooterTargetRPM(0);
    }
    private void setShooterTargetRPM(double desiredRPM) {
        targetRPM = desiredRPM * (1.0 - 0.10); // target minimum as specified (90% of desired)
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
    private void moveCarouselByDegrees(double deltaDeg){
        double newAngle = carouselAngleDeg + deltaDeg; //get New Angle
        newAngle = normalizeAngle(newAngle); // ensure within [0,360)
        carouselAngleDeg = newAngle;
        double pow = 1;
        if(deltaDeg < 0){
            carousel.setPower(-pow);
        }else {
            carousel.setPower(pow);
        }
        double timeMs = 500;
        if(119.5 <= deltaDeg && deltaDeg <= 120.5){
            timeMs *=2;
        }
        sleep(Math.round(timeMs));
        carousel.setPower(0);
        sleep(500);
    }
    /*
    private void moveCarouselByDegrees(double deltaDeg) {
        // Convert deltaDeg to encoder ticks using CAROUSEL_PPR (pulses per output shaft revolution)
        // Ensure we wait until action completes before returning
        double newAngle = carouselAngleDeg + deltaDeg;
        newAngle = normalizeAngle(newAngle); // ensure within [0,360)
        // Calculate shortest motion in terms of ticks (we will compute absolute target ticks)
        double targetRotations = newAngle / 360.0;
        double targetTicks = targetRotations * CAROUSEL_PPR;
        int targetTicksInt = (int)Math.round(targetTicks);

        // Command: run to position (using RUN_TO_POSITION)
        // We want to set current encoder to carouselAngleDeg position mapped to ticks before moving
        // But since we reset, we'll set target relative
        // Compute relative ticks from 0 to desired ticks
        carousel.setTargetPosition(targetTicksInt);
        // Use a moderate power to move (tweak as needed)
        carousel.setPower(0.5);

        // Wait until move completes
        while (opModeIsActive() && carousel.isBusy()) {
            idle();
        }
        // Stop and hold position
        carousel.setPower(0.0);
        // Store normalized angle
        carouselAngleDeg = newAngle;
    }*/

    private double normalizeAngle(double a) {
        double v = a % 360.0;
        if (v < 0) v += 360.0;
        return v;
    }
}
