package org.firstinspires.ftc.teamcode.calibration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Set0DegreesDcServoMotor", group = "Autonomous")
public class Set0DegreesDcServoMotor extends LinearOpMode {

    // --- CONFIGURE THESE FOR YOUR HARDWARE ---
    // Encoder counts per motor shaft revolution (replace with your encoder CPR)
    private static final int COUNTS_PER_MOTOR_REV = 28; // <-- replace if different
    // Gearbox ratio (motor rev : output rev). For 99.5:1 gearbox use 99.5
    private static final double GEAR_RATIO = 99.5;
    // Name in configuration
    private static final String DC_MOTOR_NAME = "carousel";
    private static final String SERVO_NAME = "pusher";
    // ----------------------------------------

    private DcMotor leftMotor = null;
    private Servo smartServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        leftMotor = hardwareMap.get(DcMotor.class, DC_MOTOR_NAME);
        smartServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // --- Setup DC motor for position control using encoder ---
        // Reset encoder and set mode to run-to-position style pattern
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optionally set zero power behavior to BRAKE for hold position
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Calculate encoder counts per degree at the output shaft:
        // countsPerOutputRev = COUNTS_PER_MOTOR_REV * GEAR_RATIO
        // countsPerDegree = countsPerOutputRev / 360
        double countsPerOutputRev = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
        double countsPerDegree = countsPerOutputRev / 360.0;

        telemetry.addData("Info", "Motor and Servo configured");
        telemetry.addData("CountsPerDegree", "%.3f", countsPerDegree);
        telemetry.update();

        // Wait for start
        waitForStart();

        // --- Set DC motor to 0 degrees ---
        // Mechanical target angle (degrees). If 0 degrees is your desired reference, targetAngle = 0.
        double targetAngleDegrees = 0.0;
        int targetTicks = (int) Math.round(targetAngleDegrees * countsPerDegree);

        // Use RUN_TO_POSITION for position control
        leftMotor.setTargetPosition(targetTicks);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a modest power to move to target; tune as needed
        leftMotor.setPower(0.3);

        // Wait until position is reached or opmode stops
        while (opModeIsActive() && leftMotor.isBusy()) {
            telemetry.addData("Motor Target (deg)", "%.1f", targetAngleDegrees);
            telemetry.addData("Current ticks", leftMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Stop motor and hold position
        leftMotor.setPower(0.0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Set Smart servo to 0 degrees ---
        // This smart servo is in Standard Mode (±150°). Map mechanical degrees (-150..+150)
        // to normalized servo position 0.0..1.0 with:
        // normalized = (angle + 150) / 300
        double servoAngleDegrees = 0.0; // desired mechanical degrees
        double servoNormalized = (servoAngleDegrees + 150.0) / 300.0; // maps to 0..1
        if (servoNormalized < 0.0) servoNormalized = 0.0;
        if (servoNormalized > 1.0) servoNormalized = 1.0;

        smartServo.setPosition(servoNormalized);

        telemetry.addData("Servo angle (deg)", "%.1f", servoAngleDegrees);
        telemetry.addData("Servo normalized", "%.3f", servoNormalized);
        telemetry.update();

        // Keep opmode alive briefly so telemetry is readable
        sleep(500);
    }
}
