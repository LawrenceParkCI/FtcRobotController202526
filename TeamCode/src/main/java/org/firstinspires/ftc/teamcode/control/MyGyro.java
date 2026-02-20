package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


public class MyGyro {
    public static double lastKnownHeading = 0.0;
    public static IMU imu;

    public static IMU createIMU(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        return imu;
    }

}
