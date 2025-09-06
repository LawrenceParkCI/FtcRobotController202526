package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Set0DegreesServoMotor", group = "Autonomous")
public class Set0DegreesServoMotor extends LinearOpMode {

    private Servo carouselServo;
    private static final double MAX_ANGLE = 150.0; // degrees each side
    private static final double FULL_RANGE = 2.0 * MAX_ANGLE; // 300 degrees

    @Override
    public void runOpMode() {
        // Initialize servo from configuration (name must match Robot Config)
        carouselServo = hardwareMap.get(Servo.class, "pusher");

        // Convert degrees to servo position 0.0 - 1.0
        double degrees = 0.0;
        double position = (degrees + MAX_ANGLE) / FULL_RANGE; // (deg + 150) / 300

        // Set initial position to 0 degrees (midpoint)
        carouselServo.setPosition(position);

        // Telemetry to confirm
        telemetry.addData("Servo target deg", degrees);
        telemetry.addData("Servo position (0..1)", position);
        telemetry.update();

        waitForStart();

        // Keep OpMode alive until stop pressed
        while (opModeIsActive()) {
            idle();
        }
    }
}