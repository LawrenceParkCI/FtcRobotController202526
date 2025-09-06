package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Set0DegreesDcMotor", group = "Autonomous")
public class Set0DegreesDcMotor extends LinearOpMode {

    private DcMotorEx carouselMotor;

    @Override
    public void runOpMode() {
        // Hardware map: change "carousel" to the name used in your Robot Configuration
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel");

        // 1) Ensure motor is in a known state
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 2) Define 0 degrees as encoder position 0 by leaving the motor physically at your desired "zero"
        //    after STOP_AND_RESET_ENCODER the encoder position is zero.

        // 3) Configure to run-to-position and hold that 0 position
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 4) Apply a small holding power so controller will actively hold position
        carouselMotor.setPower(0.2);

        waitForStart();

        // Telemetry to confirm being at zero
        while (opModeIsActive()) {
            telemetry.addData("Current ticks", carouselMotor.getCurrentPosition());
            telemetry.addData("Target ticks", carouselMotor.getTargetPosition());
            telemetry.addData("IsBusy", carouselMotor.isBusy());
            telemetry.update();

            // keep looping to hold position; stop if stop is requested
            if (isStopRequested()) break;
            sleep(50);
        }

        // Clean up: stop motor and switch to a safe mode
        carouselMotor.setPower(0.0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
