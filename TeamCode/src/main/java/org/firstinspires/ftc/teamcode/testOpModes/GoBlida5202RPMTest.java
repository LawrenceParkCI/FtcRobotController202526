package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GoBlida5202RPMTest", group = "TeleOp")
public class GoBlida5202RPMTest extends LinearOpMode {

    // Encoder specification: 28 PPR at output shaft (see goBILDA spec).
    private static final int ENCODER_PPR = 28;
    // Quadrature multiplier: if getCurrentPosition returns quadrature counts, use 4.
    // If your controller already returns "pulses" equal to PPR, set this to 1.
    private static final int QUAD_MULTIPLIER = 4;
    private static final double COUNTS_PER_REV = ENCODER_PPR * QUAD_MULTIPLIER;

    private DcMotorEx motor;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Replace "motor1" with the name configured in your robot configuration
        motor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Configure motor direction or zero power behavior as needed
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder to zero for calibration as requested
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Encoder reset to 0. Ready. Move left stick Y to run.");
        telemetry.update();

        waitForStart();

        // initial values for RPM calculation
        int lastPosition = motor.getCurrentPosition();
        timer.reset();

        while (opModeIsActive()) {
            // Read joystick (gamepad1 left stick Y). FTC convention: up is -1, down is +1.
            double joyY = -gamepad1.left_stick_y; // invert if you prefer natural forward = positive
            double clippedPower = Math.max(-1.0, Math.min(1.0, joyY));

            // Set motor power immediately based on joystick input
            motor.setPower(clippedPower);

            // Measure encoder delta and elapsed time
            int currentPosition = motor.getCurrentPosition();
            double dt = timer.seconds();
            int deltaCounts = currentPosition - lastPosition;

            double rpm = 0.0;
            if (dt > 0.0) {
                // revolutions = deltaCounts / countsPerRev
                double revolutions = ((double) deltaCounts) / COUNTS_PER_REV;
                rpm = (revolutions / dt) * 60.0;
            }

            // Telemetry: power, encoder position, delta, dt, computed RPM
            telemetry.addData("Power (joyY)", "%.3f", clippedPower);
            telemetry.addData("Encoder pos", "%d", currentPosition);
            telemetry.addData("Delta counts", "%d", deltaCounts);
            telemetry.addData("Delta time (s)", "%.3f", dt);
            telemetry.addData("RPM", "%.2f", rpm);
            telemetry.update();

            // prepare for next loop
            lastPosition = currentPosition;
            timer.reset();

            // small loop delay to avoid flooding (adjust as desired)
            sleep(50);
        }

        // Stop motor when ending
        motor.setPower(0.0);
    }
}
