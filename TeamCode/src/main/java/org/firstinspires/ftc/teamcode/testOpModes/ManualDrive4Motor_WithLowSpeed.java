package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ManualDrive4Motor_WithLowSpeed", group = "TeleOp")
public class ManualDrive4Motor_WithLowSpeed extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack  = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack  = null;

    // Configuration constants
    private static final double LOW_SPEED_MAX = 0.30; // 30% when low speed mode active
    private static final double FULL_SPEED_MAX = 1.0; // 100% otherwise
    private static final double JOYSTICK_DEADBAND = 0.05; // ignore tiny joystick noise

    @Override
    public void runOpMode() {
        // Hardware map
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Motor direction: assume right side must be reversed for standard tank orientation
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Ensure motors brake when power is zero for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joysticks
            double leftStickY = -gamepad1.left_stick_y; // forward is negative on gamepad, invert so up = positive
            double leftStickX = gamepad1.left_stick_x;  // strafe
            double rightStickX = gamepad1.right_stick_x; // rotation

            // Apply deadband
            leftStickY = applyDeadband(leftStickY, JOYSTICK_DEADBAND);
            leftStickX = applyDeadband(leftStickX, JOYSTICK_DEADBAND);
            rightStickX = applyDeadband(rightStickX, JOYSTICK_DEADBAND);

            // Low speed mode when right trigger touched (non-zero)
            boolean lowSpeedMode = gamepad1.right_trigger > 0.01;

            double maxSpeed = lowSpeedMode ? LOW_SPEED_MAX : FULL_SPEED_MAX;

            // Mecanum drive math:
            // drive = forward/back, strafe = left/right, twist = rotation
            double drive = leftStickY;
            double strafe = leftStickX;
            double twist = rightStickX;

            // Compute raw motor powers (standard mecanum wheel formula)
            double lf = drive + strafe + twist; // leftFront
            double lb = drive - strafe + twist; // leftBack
            double rf = drive - strafe - twist; // rightFront
            double rb = drive + strafe - twist; // rightBack

            // Normalize so no value exceeds 1.0
            double max = Math.max(
                    Math.max(Math.abs(lf), Math.abs(lb)),
                    Math.max(Math.abs(rf), Math.abs(rb))
            );
            if (max > 1.0) {
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;
            }

            // Apply global speed cap
            lf = lf * maxSpeed;
            lb = lb * maxSpeed;
            rf = rf * maxSpeed;
            rb = rb * maxSpeed;

            // Send to motors //adjust positve and negative depending on sign
            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);

            // Telemetry for debugging
            telemetry.addData("LowSpeed", lowSpeedMode);
            telemetry.addData("MaxSpeed", "%.2f", maxSpeed);
            telemetry.addData("LF", "%.2f", lf);
            telemetry.addData("LB", "%.2f", lb);
            telemetry.addData("RF", "%.2f", rf);
            telemetry.addData("RB", "%.2f", rb);
            telemetry.update();
        }
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        // Scale to remove the deadband gap for smoother control
        if (value > 0) return (value - deadband) / (1.0 - deadband);
        else return (value + deadband) / (1.0 - deadband);
    }
}
