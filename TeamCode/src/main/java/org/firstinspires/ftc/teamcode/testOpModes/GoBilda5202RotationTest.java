package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GoBilda5202RotationTest", group = "TeleOp")
public class GoBilda5202RotationTest extends LinearOpMode {

    private DcMotorEx carousel;
    private final double ENCODER_PPR = 2786.2; // encoder pulses per revolution at output shaft
    private double currentAngle = 0.0; // tracks angle in [0,360)
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Replace "carouselMotor" with the name you used in the robot configuration
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        // Basic motor direction / zeroing setup
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start with encoder = 0 and motor not moving
        carousel.setPower(0.0);

        waitForStart();

        // Ensure encoder is considered zeroed at start
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentAngle = 0.0;

        while (opModeIsActive()) {
            boolean actionTaken = false;

            // Read buttons
            boolean lb = gamepad2.left_bumper;
            boolean dpadRight = gamepad2.dpad_right;
            boolean dpadLeft = gamepad2.dpad_left;

            if (lb && dpadRight) {
                // LB + D-Pad R -> +60 deg
                rotateByDegrees(60.0);
                actionTaken = true;
            } else if (lb && dpadLeft) {
                // LB + D-Pad L -> -60 deg
                rotateByDegrees(-60.0);
                actionTaken = true;
            } else if (dpadRight && !lb) {
                // D-Pad R -> +120 deg
                rotateByDegrees(120.0);
                actionTaken = true;
            } else if (dpadLeft && !lb) {
                // D-Pad L -> -120 deg
                rotateByDegrees(-120.0);
                actionTaken = true;
            }

            // If no action, ensure motor remains with 0 power and do nothing
            if (!actionTaken) {
                carousel.setPower(0.0);
            }

            // tiny sleep so loop isn't spinning too tight
            sleep(20);
        }
    }

    /**
     * Rotates the motor by deltaDegrees relative to the currentAngle.
     * Ensures final angle is normalized into [0,360).
     * Moves the motor using RUN_TO_POSITION, waits until movement completes,
     * then sets motor power to 0 to "stop it there at power of 0".
     */
    private void rotateByDegrees(double deltaDegrees) throws InterruptedException {
        // Compute target angle, normalized into [0,360)
        double newAngle = normalizeAngle(currentAngle + deltaDegrees);

        // Compute delta from encoder perspective (we use shortest signed difference on cumulative ticks)
        double degreesToMove = angleDeltaSigned(currentAngle, newAngle);

        int ticksToMove = degreesToTicks(degreesToMove);

        // Current encoder position
        int startTicks = carousel.getCurrentPosition();
        int targetTicks = startTicks + ticksToMove;

        // Setup RUN_TO_POSITION
        carousel.setTargetPosition(targetTicks);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to move (positive for forward movement, negative for reverse)
        double movePower = (ticksToMove >= 0) ? 0.5 : -0.5;
        // Use a reasonable power; tune if you need faster/slower movement
        carousel.setPower(movePower);

        // Wait until motion completes or opMode is no longer active
        while (opModeIsActive() && carousel.isBusy()) {
            // Optionally, add timeout protection if desired
            sleep(10);
        }

        // Stop motor and set power to 0 as requested
        carousel.setPower(0.0);

        // Keep the encoder reading as-is so future calculations are relative to actual position
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update currentAngle to the normalized newAngle
        currentAngle = newAngle;
    }

    /**
     * Convert signed degrees to encoder ticks (signed). Positive means clockwise in this scheme.
     */
    private int degreesToTicks(double degrees) {
        double ticksPerRev = ENCODER_PPR; // encoder resolution at output shaft
        double ticks = (ticksPerRev * degrees) / 360.0;
        return (int) Math.round(ticks);
    }

    /**
     * Normalize angle into [0,360)
     */
    private double normalizeAngle(double angle) {
        double a = angle % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    /**
     * Compute the signed minimal delta (degrees) from angleFrom to angleTo
     * returning a value in (-180, 180] range, but we want the commanded delta
     * to be exactly the arithmetic difference specified by user inputs (±60 or ±120).
     * For this OpMode we want the exact delta (not shortest path), so we compute direct delta.
     */
    private double angleDeltaSigned(double from, double to) {
        // Direct arithmetic delta (to - from), then normalize to the signed range (-360,360)
        double raw = to - from;
        // We want the exact delta that was commanded (e.g., +60 or -120) rather than always the shortest rotation.
        // But if user presses commands repeatedly, currentAngle has been normalized and deltaDegrees will be the intended signed change.
        // Here we return raw, but ensure it's between -360 and +360 for safety.
        if (raw > 360.0) raw = raw % 360.0;
        if (raw < -360.0) raw = raw % 360.0;
        return raw;
    }
}
