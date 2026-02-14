package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


import android.graphics.Color;

@Autonomous(name = "ColorSensorTest", group = "Autonomous")
public class ColorSensorTest extends LinearOpMode {

    private NormalizedColorSensor colorSensor;


    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map (name: "colorSensor")
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        telemetry.clear();
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            // Telemetry output based on detection
            if (isColorGreen()) {
                telemetry.addData("Detected Color", "green");
            } else if (isColorPurple()) {
                telemetry.addData("Detected Color", "purple");
            } else {
                telemetry.addData("Detected Color", "not green and not purple");
            }

            // Helpful debug values
            telemetry.addData("Hue", "%.1f", readColor()[0]);
            telemetry.update();

        }
    }
    private float[] readColor(){
        // Read normalized RGBA values
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        // Convert to HSV
        Color.colorToHSV(colors.toColor(), hsv);

        return hsv;
    }
    private boolean isColorGreen(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 120 && hue <= 180);
    }
    private boolean isColorPurple(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 181 && hue <= 245);
    }
}
