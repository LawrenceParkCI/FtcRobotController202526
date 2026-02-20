package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.control.*;

//Blue
@Autonomous(name="AutoBlueShoot3BallNoCamera", group="Autonomous")
public class AutoDrive4MotorRotateBlueShootBallWithoutCamera extends LinearOpMode {

    private int startDelay = 0;
    private int delay = 0; //ms delay before backing up
    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Mechanism motors
    private DcMotor intake;
    private Shooter shooter;
    private Carousel carousel;

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

        long start = System.currentTimeMillis();
        //time is for delay before backing up
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < startDelay){
            mainDo();
        }
        // Drive forward for total 3.1s
        driveForwardFixedTime(0.7, 1);
        sleep(200);
        shoot(2500);
        carousel.rotateThirdLeft();
        while(!carousel.isFinished() && opModeIsActive()){sleep(50);}
        shoot(2500);
        carousel.rotateThirdLeft();
        while(!carousel.isFinished() && opModeIsActive()){sleep(50);}
        shoot(2500);
        sleep(200);
        rotateFixedTime(0.5, 1);
        sleep(200);
        start = System.currentTimeMillis();
        //time is for delay before backing up
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < delay){
            mainDo();
        }
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

        carousel = new Carousel(hardwareMap, Carousel.AUTO);
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


    //Shoots at target rpm
    private void shoot(double RPM){
        shooter.start(RPM);
        while(opModeIsActive() && !shooter.isTargetMet()){
            //TODO ensure still facing AprilTag on Goal
            mainDo();
        }
        shooter.push();

        //pause 200 ms
        long currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 300){
            mainDo();
        }
        shooter.stop();
    }

    private void mainDo(){
        shooter.updateRPM();
        updateTelemetry();
    }
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Shooter speed: ", shooter.getMotor().getVelocity());
        telemetry.update();
    }

}
