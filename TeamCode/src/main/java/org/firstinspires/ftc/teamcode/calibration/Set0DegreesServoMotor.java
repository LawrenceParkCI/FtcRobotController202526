package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Set0DegreesServoMotor", group = "Autonomous")
public class Set0DegreesServoMotor extends LinearOpMode {

    private Servo servo;
    private static final double SERVO_FULL_RANGE_DEG = 300; // 300 degrees

    @Override
    public void runOpMode() {
        // Initialize servo from configuration (name must match Robot Config)
        servo = hardwareMap.get(Servo.class, "pusher");

        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.addData("Servo Positon", servo.getPosition());
        telemetry.update();
        waitForStart();
        setServoAngle(servo, 0);
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.addData("Servo Positon", servo.getPosition());
        telemetry.update();
        // Keep OpMode alive until stop pressed
        while (opModeIsActive()) {

        }
    }
    private void setServoAngle(Servo s, double angleDeg) {
        // Map 0..SERVO_FULL_RANGE_DEG to 0..1 position
        double pos = RangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }
    private double RangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
