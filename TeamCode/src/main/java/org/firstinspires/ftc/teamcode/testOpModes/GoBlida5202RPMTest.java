package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "GoBlida5202RPMTest", group = "TeleOp")
public class GoBlida5202RPMTest extends LinearOpMode {

    // Encoder specification: 28 PPR at output shaft (see goBILDA spec).
    private static final double SHOOTER_PPR = 28.0;
    private static double currentRPM = 0;
    private DcMotorEx motor;

    @Override
    public void runOpMode() {

        // Replace "motor1" with the name configured in your robot configuration
        motor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Configure motor direction or zero power behavior as needed
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry.addLine("Ready. Move left stick Y to run.");
        telemetry.update();
        waitForStart();
        setShooterTargetRPM(5000);
        updateShooterRPM();
        motor.setCurrentAlert(9.25, CurrentUnit.AMPS);
        while (opModeIsActive()) {

            updateShooterRPM();
            telemetry.clearAll();
            telemetry.addData("Current RPM: ", currentRPM);
            telemetry.addData("Current (A): ", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current (A): ", motor.isOverCurrent());
            telemetry.addData("Power: ", motor.getPower());
            telemetry.update();
        }

    }
    private void setShooterTargetRPM(double desiredRPM) {
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        // DcMotorEx allows setting velocity in ticks per second
        motor.setVelocity(ticksPerSec);
        // Immediately after changing speed, update actual currentRPM from encoder
        updateShooterRPM();
    }

    private void updateShooterRPM() {
        //get ticks per second
        double ticksPerSec = motor.getVelocity();
        //update current RPM ticksPerSec*RPM -> RPM
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
    }
}
