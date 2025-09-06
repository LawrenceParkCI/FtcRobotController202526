package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp mode for an FTC chassis with two wheels.
 * A single joystick (gamepad1.left_stick) is used:
 *   - Pushing up (negative y) moves the robot forward.
 *   - Pushing down moves it backward.
 *   - Moving left/right turns the robot.
 *
 * Telemetry (elapsed time, speed in m/s, acceleration in m/s²,
 * left/right motor power) is sent both to the driver station and
 * via the REV hardware client to the FTC Log Viewer.
 */
@TeleOp(name="ChassisJoystickTwoMotor", group="TeleOp")
public class ChassisJoystickTwoMotor extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    // Previous encoder positions and time for speed/acceleration calculations.
    private int prevLeftPos = 0;
    private int prevRightPos = 0;
    private double prevTime = 0;
    private double prevSpeed = 0; // average speed of both motors

    // Constants for converting encoder ticks to distance (meters)
    private static final double TICKS_PER_REV = 1120.0; // Adjust as needed for your motor
    private static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches in meters approx.
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_METERS;
    private static final double TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    @Override
    public void init() {
        // Retrieve motors from the hardware map by name.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Set motor directions. (You may need to reverse one of them depending on your build.)
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset and configure encoders.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        prevTime = runtime.seconds();
        prevLeftPos = leftMotor.getCurrentPosition();
        prevRightPos = rightMotor.getCurrentPosition();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Joystick Control Calculation ---
        // Using gamepad1.left_stick: y controls forward/backward, x controls turning.
        // (Note: The y-axis is inverted on the joystick—pushed up is negative)
        double drive = -gamepad1.left_stick_y; // forward/backward component
        double turn  = gamepad1.left_stick_x;    // turning component

        // Combine drive and turn into power for each motor.
        double leftPower  = drive + turn;
        double rightPower = drive - turn;

        // Normalize the powers so neither exceeds 1.0.
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower  /= maxPower;
            rightPower /= maxPower;
        }

        // Set motor powers.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // --- Telemetry Calculation ---
        // Calculate elapsed time and time interval since the last loop.
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - prevTime;
        if (deltaTime <= 0) deltaTime = 0.001;  // safeguard against division by zero

        // Read current encoder positions.
        int currentLeftPos  = leftMotor.getCurrentPosition();
        int currentRightPos = rightMotor.getCurrentPosition();

        // Compute change in encoder counts.
        int deltaLeftTicks  = currentLeftPos - prevLeftPos;
        int deltaRightTicks = currentRightPos - prevRightPos;

        // Convert ticks to distance (meters). Average the distances.
        double leftDistance  = deltaLeftTicks / TICKS_PER_METER;
        double rightDistance = deltaRightTicks / TICKS_PER_METER;
        double distance = (leftDistance + rightDistance) / 2.0;

        // Calculate speed in meters per second.
        double speed = distance / deltaTime;

        // Calculate acceleration (change in speed over time).
        double acceleration = (speed - prevSpeed) / deltaTime;

        // Output telemetry data.
        telemetry.addData("Elapsed Time (s)", String.format("%.2f", currentTime));
        telemetry.addData("Speed (m/s)", String.format("%.2f", speed));
        telemetry.addData("Acceleration (m/s²)", String.format("%.2f", acceleration));
        telemetry.addData("Left Motor Power", String.format("%.2f", leftPower));
        telemetry.addData("Right Motor Power", String.format("%.2f", rightPower));
        telemetry.update();

        // The REV hardware client (FTC Log Viewer) automatically picks up telemetry
        // from telemetry.addData() calls if configured. This will send all telemetry data
        // (elapsed time, speed, acceleration, motor powers) to the dashboard.

        // --- Prepare for next loop iteration ---
        prevTime = currentTime;
        prevLeftPos = currentLeftPos;
        prevRightPos = currentRightPos;
        prevSpeed = speed;
    }
}