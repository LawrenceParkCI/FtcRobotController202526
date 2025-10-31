package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ServoPosSweepStopGuaranteed", group = "Autonomous")
public class ServoPosSweepStopGuaranteed extends LinearOpMode {

    private Servo carouselServo;

    // Convert degrees in range [-150, +150] to servo position [0.0, 1.0]
    private double degreesToPosition(double degrees) {
        return (degrees + 150.0) / 300.0;
    }

    @Override
    public void runOpMode() {
        // Change the string below to match the servo name in your robot configuration
        carouselServo = hardwareMap.get(Servo.class, "carouselServo");

        // Safety: set initial position to 0 degrees (converted)
        double pos0 = degreesToPosition(0.0);    // should be 0.5 for ±150° mapping
        double pos80 = degreesToPosition(80.0);  // should be 0.7 for ±150° mapping

        carouselServo.setPosition(pos0);

        waitForStart();

        // Repeat the cycle 5 times
        for (int i = 0; i < 5 && opModeIsActive(); i++) {

            // Set to 0 degrees and hold
            carouselServo.setPosition(pos0);
            // "Holding at power 0" is not applicable to positional servo in SDK;
            // keeping position call ensures it stays commanded at that angle.
            sleep(5000);

            // Set to 80 degrees and hold
            carouselServo.setPosition(pos80);
            sleep(5000);
        }

        // Final: set back to 0 degrees and hold
        carouselServo.setPosition(pos0);

        // Optionally idle until stop or short delay to let servo reach position
        sleep(500);
    }
}