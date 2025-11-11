package org.firstinspires.ftc.teamcode.testOpModes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ManualDrive2Motor_WithLowSpeed", group="TeleOp")
public class ManualDrive2Motor_WithLowSpeed extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Reverse one motor if needed depending on wiring
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // --- Joystick inputs ---
            double driveY = -gamepad1.left_stick_y;   // forward/back
            double strafeX = gamepad1.left_stick_x;   // left/right strafe
            double spinX = gamepad1.right_stick_x;    // spin in place

            // --- Speed mode ---
            boolean lowSpeedMode = gamepad1.right_trigger > 0.1;
            double speedScale = lowSpeedMode ? 0.3 : 1.0;

            // --- Motor power calculation ---
            // Forward/back + strafe + spin combined
            double leftPower  = driveY + strafeX - spinX;
            double rightPower = driveY - strafeX + spinX;

            // Normalize to keep values within [-1,1]
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Apply speed scaling
            leftMotor.setPower(leftPower * speedScale);
            rightMotor.setPower(rightPower * speedScale);

            // --- Telemetry ---
            telemetry.addData("Left Power", leftPower * speedScale);
            telemetry.addData("Right Power", rightPower * speedScale);
            telemetry.addData("Low Speed Mode", lowSpeedMode);
            telemetry.update();
        }
    }
}

