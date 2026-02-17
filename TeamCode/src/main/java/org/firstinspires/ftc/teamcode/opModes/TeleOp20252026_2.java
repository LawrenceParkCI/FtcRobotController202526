package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Color;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp20252026_2", group = "TeleOp")
public class TeleOp20252026_2 extends LinearOpMode {
    // Drive motors (Control Hub)
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private IMU imu;
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
    private static final double CAROUSEL_PPR3rd = CAROUSEL_PPR/3;
    private static final double CAROUSEL_PPR6th = CAROUSEL_PPR/6;
    private static double tol = 0.0;
    private static int current = 0;
    // Servo angle mapping if servo range is 300 degrees (±150) in standard mode.
    // Map 0..300 degrees -> 0.0..1.0 (adjust if your servo API expects different)
    private static final double SERVO_FULL_RANGE_DEG = 300.0;
    // Carousel current angle in degrees [0,360)
    private int carouselAngleDeg = 0;
    //Intake State
    private boolean intakeActive = false;
    private long currTimeIntake=0;

    private int target = 0;
    //Shooter state

    private boolean servoActive = false;
    private boolean rpmShooterHold = false;
    private boolean shooterActive = false;
    private long currTimeShooter= 0;
    //Carousel state
    private boolean rotateActive = false;
    private long currPos = 0;

    private double required = 0;

    private NormalizedColorSensor colorSensorFront;
    private NormalizedColorSensor colorSensorBack;

    private char[] colors = {'a', 'a', 'a'};
    private int colorInt = 0;
    private float h = 0;
    private float s = 0;
    private char front = 'a';
    private char back = 'a';

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.clear();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();
        waitForStart();
        // Main loop
        while (opModeIsActive()) {
            // --- Read Colors ---
            readColor();

            drive();
            // --- INTAKE CONTROL on shooter (gamepad1) ---
            if(gamepad1.y && !intakeActive){
                double RPM = -3000;
                // Compute ticks per second for desired rpm (we set motor velocity to achieve the desired RPM)
                double ticksPerSec = RPM * SHOOTER_PPR / 60.0;
                // DcMotorEx allows setting velocity in ticks per second
                intakeActive = true;
                shooter.setVelocity(ticksPerSec);
                currTimeIntake = System.currentTimeMillis();
            }
            doAll();
            if(intakeActive && System.currentTimeMillis() - currTimeIntake >= 1750){
                intakeActive = false;
                shooter.setVelocity(0);
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
            // --- SHOOTER CONTROL with servo (gamepad2) ---
            // Buttons: X=1000 RPM, A=2500 RPM, B=4000 RPM, shoot and stop motor once done
            // When pressed, set motor velocity and update currentRPM and targetRPM logic.

            // We'll set shooter velocity in ticks per second, using SHOOTER_PPR pulses per rev:
            // desiredRPM -> ticksPerSec = desiredRPM * (SHOOTER_PPR / 60)

            doAll();
//            if (gamepad2.x && !shooterActive) {
//                setDcMotorRPM(1000);
//                shooterActive = true;
//                servoActive = false;
//            }
            if (gamepad2.a && !shooterActive) {
                setDcMotorRPM(2500);
                shooterActive = true;
                servoActive = false;
            }
            if (gamepad2.b && !shooterActive) {
                setDcMotorRPM(4500);
                shooterActive = true;
                servoActive = false;
            }
            if(gamepad2.y && shooterActive){
                shooterActive = false;
                stopDcMotor();
            }
            doAll();
            if(shooterActive){
                doAll();
                if(targetMet && !rpmShooterHold){
                    rpmShooterHold = true;
                    servoActive = true;
                    currTimeShooter = System.currentTimeMillis();
                    setServoAngle(pusher, 80.0);
                }

            }
            doAll();
            if(servoActive && System.currentTimeMillis() - currTimeShooter >= 200){
                setServoAngle(pusher, 0.0);
                stopDcMotor();
                shooterActive = false;
                rpmShooterHold = false;
                servoActive = false;
                colors[1] = 'a';

            }
            doAll();
            // --- CAROUSEL CONTROL (gamepad2) ---
            //reset current position
            if(gamepad2.back){
                current = carousel.getCurrentPosition();
            }

            // Commands require waiting until the rotation is complete before processing next.
            //checks input from controller
            if(!rotateActive) {
                manualCarousel();
                autoCarousel();
            }


            // Check if move is complete
            if (rotateActive && !carousel.isBusy() ) {
                rotateActive = false;
                // Switch back so analog triggers still work
                carousel.setPower(0);
                carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            //cancels rotate
            if(rotateActive && gamepad2.xWasPressed()){
                rotateActive = false;
                carousel.setPower(0);
                carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

//            carouselAngleDeg += rotateDegree;
//            carouselAngleDeg = normalizeAngle(carouselAngleDeg);

            doAll();
        }
    }

    public void rotateCarousel(int amt){
        int newPosition = (int) (amt) + current;
        current = newPosition;
        carousel.setTargetPosition(newPosition);
        carousel.setTargetPositionTolerance(5);
        carousel.setPower(0.8);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateActive = true;
    }
    private void autoCarousel(){
        char c = colors[0];
        //DPad Right/Left -> ±120°
        int rotateToShooter = 0;

        if (gamepad2.rightBumperWasPressed()){

            if(colorInt != 1 && colorInt != -1){
                rotateToShooter = (int) CAROUSEL_PPR6th;
                colorInt++;
            }
            if(colors[1] == 'p'){
                rotateCarousel(-rotateToShooter);
            }else if (colors[2] == 'p'){
                rotateCarousel((int)CAROUSEL_PPR3rd - rotateToShooter);
                colors[0] = colors[1];
                colors[1] = colors[2];
                colors[2] = c;
            } else if(colors[0] == 'p'){
                rotateCarousel(-(int)CAROUSEL_PPR3rd - rotateToShooter);
                colors[0] = colors[2];
                colors[2] = colors[1];
                colors[1] = c;
            }
        }
        if (gamepad2.leftBumperWasPressed()){

            if(colorInt != 1 && colorInt != -1){
                rotateToShooter = (int) CAROUSEL_PPR6th;
                colorInt++;
            }
            if(colors[1] == 'g'){
                rotateCarousel(-rotateToShooter);
            }else if (colors[2] == 'g'){
                rotateCarousel((int)CAROUSEL_PPR3rd - rotateToShooter);
                colors[0] = colors[1];
                colors[1] = colors[2];
                colors[2] = c;
            } else if(colors[0] == 'g'){
                rotateCarousel(-(int)CAROUSEL_PPR3rd - rotateToShooter);
                colors[0] = colors[2];
                colors[2] = colors[1];
                colors[1] = c;
            }
        }

        if (gamepad2.dpad_right) {
            rotateCarousel(-(int)CAROUSEL_PPR3rd);
            colors[0] = colors[2];
            colors[2] = colors[1];
            colors[1] = c;
        }
        if (gamepad2.dpad_left) {
            rotateCarousel((int)CAROUSEL_PPR3rd);
            colors[0] = colors[1];
            colors[1] = colors[2];
            colors[2] = c;
        }
        if(gamepad2.dpad_up) {
            rotateCarousel(-(int)CAROUSEL_PPR6th);
            colorInt++;
            if(colorInt == 2){
                colorInt = 0;
            }
        }
        if(gamepad2.dpad_down){
            rotateCarousel((int)CAROUSEL_PPR6th);
            colorInt--;
            if(colorInt == -2){
                colorInt = 0;
            }
        }
    }
    private void manualCarousel() {
        double carouselPower = 0.0;
        if (gamepad2.right_trigger > 0.05) {
            carouselPower = -gamepad2.right_trigger / 2; // backwards full
            current = carousel.getCurrentPosition(); //changes the current position with the manual rotate 
        } else if (gamepad2.left_trigger > 0.05) {
            carouselPower = gamepad2.left_trigger / 2; // forwards
            current = carousel.getCurrentPosition(); //changes the current position with the manual rotate
        } else {
            carouselPower = 0.0;
        }

//        if (gamepad2.right_bumper) {
//            carouselPower *= 0.3;
//        }
        carousel.setPower(carouselPower);
    }

    private void initHardware(){
        // --- Hardware mapping ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");

        current = carousel.getCurrentPosition();

        pusher = hardwareMap.get(Servo.class, "pusher");
        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFront");
        colorSensorBack = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");
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
        setServoAngle(pusher, 0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselAngleDeg = 0; // encoder is zeroed above
    }
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Carousel Degree: ", carouselAngleDeg);
        telemetry.addData("ColorFront", front);
        telemetry.addData("ColorBack", back);
        telemetry.addData("Color 1", colors[0]);
        telemetry.addData("Color 2", colors[1]);
        telemetry.addData("Color 3", colors[2]);
        telemetry.addData("Hue", h);
        telemetry.addData("saturation", s);
        telemetry.addData("Current Shooter RPM:", currentRPM);
        telemetry.addData("Current Target RPM:", String.format("%.1f", targetRPM));
        telemetry.addData("Current Amperage ", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Target RPM Met?: ", targetMet);
        telemetry.addData("Carousel Positon", carousel.getCurrentPosition());
        telemetry.addData("Carousel Power", carousel.getPower());
        telemetry.addData("Carousel Active", rotateActive);
        telemetry.addData("Last Position", currPos);
        telemetry.addData("Target: ", target);
        telemetry.addData("Carousel Delta", carousel.getCurrentPosition() - currPos);
        telemetry.addData("Carousel Required", required);
        telemetry.addData("Carousel AMP", carousel.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    // --- Helper methods ---

    private void drive(){
        // --- DRIVE CONTROL (gamepad1) ---
        double lx = gamepad1.left_stick_x;   // left-stick left/right: strafing
        double ly = -gamepad1.left_stick_y;   // left-stick up/down: forward/back
        double rx = -gamepad1.right_stick_x;  // right-stick left/right: rotation

        if(gamepad1.bWasPressed()){
            imu.resetYaw();
        }
        // Compute base motion powers
        // Mecanum drive mixing: forward/back = ly, strafe = lx, rotate = rx
        double theta = Math.atan2(ly, lx);
        double r = Math.hypot(lx, ly);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        ly = r * Math.sin(theta);
        lx = r * Math.cos(theta);

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
        double speedLimit = gamepad1.right_trigger > 0.05 ? 0.50 : 1.0;
        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }
    private void setServoAngle(Servo s, double angleDeg) {
        // Map 0..SERVO_FULL_RANGE_DEG to 0..1 position
        double pos = rangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }
    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    private void setDcMotorRPM(double desiredRPM) {
        targetRPM = desiredRPM; // target minimum as specified
        // Compute ticks per second for desired rpm (we set motor velocity to achieve the desired RPM)
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        // DcMotorEx allows setting velocity in ticks per second
        shooter.setVelocity(ticksPerSec);
        doAll();
    }
    private void stopDcMotor(){
        setDcMotorRPM( 0);
    }
    private void updateDcMotorRPM() {
        //get ticks per second
        double ticksPerSec = shooter.getVelocity();
        //update current RPM ticksPerSec*RPM -> RPM
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
        targetMet = currentRPM >= targetRPM;
    }
    private int normalizeAngle(int a) {
        int v = a % 360;
        if (v < 0) v += 360.0;
        return v;
    }
    private void readColor(){
        // Read normalized RGBA values
        NormalizedRGBA colorsfront = colorSensorFront.getNormalizedColors();
        float[] hsv1 = new float[3];
        // Convert to HSV
        Color.colorToHSV(colorsfront.toColor(), hsv1);

        NormalizedRGBA colorsBack = colorSensorBack.getNormalizedColors();
        float[] hsv2 = new float[3];
        // Convert to HSV
        Color.colorToHSV(colorsBack.toColor(), hsv2);
        if(isColorGreen(hsv1)){
            front = 'g';
            colors[0] = front;
        } else if(isColorPurple(hsv1)){
            front = 'p';
            colors[0] = front;
        }else{
            front = 'a';
        }
        if(isColorGreen(hsv2)){
            back = 'g';
        } else if(isColorPurple(hsv2)){
            back = 'p';
        }else{
            back = 'a';
        }

        h = hsv1[0];
        s = hsv1[1];

    }
    private boolean isColorGreen(float[] hsv){
        double hue = hsv[0];
        return (hue >= 120 && hue <= 160);
    }
    private boolean isColorPurple(float[] hsv){
        double hue = hsv[0];
        return (hue >= 181 && hue <= 245);
    }
    private void doAll(){
        updateDcMotorRPM();
        updateTelemetry();
    }
}