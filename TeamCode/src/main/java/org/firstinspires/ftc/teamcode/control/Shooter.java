package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Encoder specs (from manufacturer data)
// Shooter motor reference: goBILDA 5202 1:1, 6000 rpm
// Shooter 5202 motor (1:1) -> 28 pulses per motor revolution at output shaft.

// Servo reference: Studica Multi-Mode Smart Servo (Standard Mode)
// Servo angle mapping if servo range is 300 degrees (±150) in standard mode.
// Map 0..300 degrees -> 0.0..1.0 (adjust if your servo API expects different)
public class Shooter {

    private DcMotorEx shooter;
    private Servo pusher;

    // Encoder specs (from manufacturer data)
    // Shooter 5202 motor (1:1) -> 28 pulses per motor revolution at output shaft.
    public static final double SHOOTER_PPR = 28.0;
    // Servo angle mapping if servo range is 300 degrees (±150) in standard mode.
    // Map 0..300 degrees -> 0.0..1.0 (adjust if your servo API expects different)
    private static final double SERVO_FULL_RANGE_DEG = 300.0;


    // Shooter RPM tracking
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;

    private boolean shooterActive;
    private boolean pusherActive;
    private boolean rpmShooterHold;

    // Shooter motor reference: goBILDA 5202 1:1, 6000 rpm
    // Servo reference: Studica Multi-Mode Smart Servo (Standard Mode)
    public Shooter(HardwareMap hardwareMap){
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        // Shooter and carousel: set mode
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize pusher servo to 0 degrees (calibrated start)
        setPusherAngle(0.0);
    }

    public void setPusherAngle(double angleDeg) {
        // Map 0..SERVO_FULL_RANGE_DEG to 0..1 position
        double pos = RangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        pusher.setPosition(pos);
    }

    private double RangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void setTargetRPM(double desiredRPM) {
        targetRPM = desiredRPM; // target minimum as specified
        // Compute ticks per second for desired rpm (we set motor velocity to achieve the desired RPM)
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        // DcMotorEx allows setting velocity in ticks per second
        shooter.setVelocity(ticksPerSec);
        // Immediately after changing speed, update actual currentRPM from encoder
        updateRPM();
    }

    /**
     * Updates the current RPM of the shooter
     * @return Returns true when the curent RPM is greater than
     * or equal to the target RPM.
     */
    public boolean updateRPM() {
        //get ticks per second
        double ticksPerSec = shooter.getVelocity();
        //update current RPM ticksPerSec*RPM -> RPM
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
        return isTargetMet();
    }

    public void start(double desiredRPM){
        setTargetRPM(desiredRPM);
        shooterActive = true;
        pusherActive = false;
    }

    public void stop(){
        setTargetRPM(0);
        setPusherAngle(0);
        shooterActive = false;
        pusherActive = false;
    }

    public void push(){
        rpmShooterHold = true;
        pusherActive = true;
        setPusherAngle(80.0);
    }

    public boolean isTargetMet(){
        return currentRPM >= targetRPM;
    }

    public double getCurrentRPM(){
        return currentRPM;
    }

    public boolean isShooterActive(){
        return shooterActive;
    }

    public boolean isPusherActive(){
        return pusherActive;
    }

    public DcMotorEx getMotor(){
        return shooter;
    }

    public Object getTargetRPM() {
        return targetRPM;
    }
}
