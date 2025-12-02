package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="ColorSensorTest", group="Autonomous")
public class ColorSensorTest extends LinearOpMode {

    // Declare the color sensor
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize the hardware map to get the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Tell driver station that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Read color values
            int red   = colorSensor.red();
            int green = colorSensor.green();
            int blue  = colorSensor.blue();
            int alpha = colorSensor.alpha(); // overall light intensity
            int argb = colorSensor.argb();
            // Send data to telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("RGB", argb);
            telemetry.update();

            // Idle to prevent CPU overuse
            idle();
        }
    }
}

