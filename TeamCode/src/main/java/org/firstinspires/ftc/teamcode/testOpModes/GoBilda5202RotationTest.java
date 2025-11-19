package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GoBilda5202RotationTest", group="TeleOp")
public class GoBilda5202RotationTest extends LinearOpMode {

    // IMPORTANT: Replace this with the correct ticks-per-output-rotation for your motor/gearbox.
    // For goBILDA 5202 Yellow Jacket, TPR depends on the gear ratio.
    // Example placeholders (do NOT assume they match your 99.5:1 model):
    //  - 13.7:1  = ~537.6
    //  - 19.2:1  = ~753.6
    //  - 30:1    = ~1600
    //  - 50:1    = ~2800
    //  - 99.5:1  = ~5600–11200 (varies with encoder CPR assumptions)
    // Verify your exact `TICKS_PER_REV` from your motor specs or by calibration.
    private static final double TICKS_PER_REV = 5600; // placeholder; measure and update!

    // Motion power used for RUN_TO_POSITION moves (tune as needed)
    private static final double MOVE_POWER = 0.4;

    // Angle increments
    private static final double SIXTH_TURN_DEG = 60.0;
    private static final double THIRD_TURN_DEG = 120.0;

    // Debounce timing to avoid double-triggering on a single press
    private static final double DEBOUNCE_MS = 200;

    // Track angle in [0, 360)
    private double currentAngleDeg = 0.0;

    // Edge detection helpers
    private boolean prevLB = false;
    private boolean prevDpadL = false;
    private boolean prevDpadR = false;

    private DcMotorEx motor;
    private ElapsedTime pressTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the motor from the configuration as "carousel" (rename if needed)
        motor = hardwareMap.get(DcMotorEx.class, "carousel");

        // Ensure no drift when power = 0
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set encoder to zero and establish 0° reference
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
        currentAngleDeg = 0.0; // angle in [0, 360)
        pressTimer.reset();

        telemetry.addLine("Initialized: encoder = 0, angle = 0°");
        telemetry.addLine("Controls (gamepad2):");
        telemetry.addLine("  LB + Dpad Right: +60° (clockwise)");
        telemetry.addLine("  LB + Dpad Left : -60° (counterclockwise)");
        telemetry.addLine("  Dpad Right     : +120° (clockwise)");
        telemetry.addLine("  Dpad Left      : -120° (counterclockwise)");
        telemetry.addLine("Motor frozen when idle");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Keep motor frozen unless a command is executing
            // RUN_TO_POSITION holds itself; when idle ensure power 0 and target=current
            if (!motor.isBusy()) {
                motor.setPower(0.0);
                motor.setTargetPosition(motor.getCurrentPosition());
            }

            // Read current button states
            boolean lb = gamepad2.left_bumper;
            boolean dpadL = gamepad2.dpad_left;
            boolean dpadR = gamepad2.dpad_right;

            // Edge detection (press events)
            boolean lbPressed   = lb && !prevLB;
            boolean dpadLPressed = dpadL && !prevDpadL;
            boolean dpadRPressed = dpadR && !prevDpadR;

            // Update previous states
            prevLB = lb;
            prevDpadL = dpadL;
            prevDpadR = dpadR;

            // Only accept a new command if:
            //  - Motor not busy (previous action completed)
            //  - Debounce interval elapsed
            boolean readyForNewCommand = !motor.isBusy() && (pressTimer.milliseconds() > DEBOUNCE_MS);

            if (readyForNewCommand) {
                Double deltaDeg = null;

                // Priority order per spec:
                // If LB + Dpad R -> +60°
                // Else if LB + Dpad L -> -60°
                // Else if Dpad R -> +120°
                // Else if Dpad L -> -120°
                // Else -> do nothing (freeze)

                if (lb && dpadRPressed) {
                    deltaDeg = +SIXTH_TURN_DEG;
                } else if (lb && dpadLPressed) {
                    deltaDeg = -SIXTH_TURN_DEG;
                } else if (!lb && dpadRPressed) {
                    deltaDeg = +THIRD_TURN_DEG;
                } else if (!lb && dpadLPressed) {
                    deltaDeg = -THIRD_TURN_DEG;
                }

                if (deltaDeg != null) {
                    // Compute new angle and wrap to [0, 360)
                    currentAngleDeg = wrapAngle360(currentAngleDeg + deltaDeg);

                    // Convert angle to absolute ticks relative to the initial zero reference
                    int targetTicks = angleDegToTicks(currentAngleDeg);

                    // Command the move and wait for completion
                    moveToTargetTicksBlocking(targetTicks);

                    // Reset debounce timer after a completed command
                    pressTimer.reset();
                }
            }

            // Telemetry
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.addData("Busy", motor.isBusy());
            telemetry.addData("Angle°", currentAngleDeg);
            telemetry.update();

            idle();
        }
    }

    // Convert an angle in [0,360) to ticks on the output shaft relative to initial zero
    private int angleDegToTicks(double angleDeg) {
        double ticks = (angleDeg / 360.0) * TICKS_PER_REV;
        return (int) Math.round(ticks);
    }

    // Execute a RUN_TO_POSITION move and block until complete, then freeze at 0 power
    private void moveToTargetTicksBlocking(int targetTicks) throws InterruptedException {
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(MOVE_POWER);

        // Wait until motion completes or stop requested
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Moving to", targetTicks);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Freeze at final position
        motor.setPower(0.0);
        motor.setTargetPosition(motor.getCurrentPosition());
    }

    // Wrap angle to [0, 360)
    private double wrapAngle360(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }
}
