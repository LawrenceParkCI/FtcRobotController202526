package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {

    /**
     * RUN_TO_POSITION
     */
    public static final int AUTO = 0;
    /**
     * RUN_USING_ENCODER
     */
    public static final int MANUAL = 1;

    private DcMotorEx carousel;      // GoBilda 60 RPM gearbox (99.5:1) - encoder used for angle control
    private int position = 0;
    private static final double CAROUSEL_PPR = 2786.2;
    private static final double CAROUSEL_PPR3rd = CAROUSEL_PPR/3;
    private static final double CAROUSEL_PPR6th = CAROUSEL_PPR/6;
    private boolean rotateActive = false;

    //odd indexes are shooting positions
    //even indexes are intake positions
    private char[] balls = {'a', ' ', 'a', ' ', 'a', ' '};


    public Carousel(HardwareMap hardwareMap, int mode){
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode(mode);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        position = 0; // encoder is zeroed above
    }

    public DcMotorEx getMotor(){
        return carousel;
    }

    public void rotateThirdRight(){
        rotateCarousel(-(int)CAROUSEL_PPR3rd);
    }
    public void rotateThirdLeft(){
        rotateCarousel((int)CAROUSEL_PPR3rd);
    }

    public void rotateSixthRight(){
        rotateCarousel(-(int)CAROUSEL_PPR6th);
    }
    public void rotateSixthLeft(){
        rotateCarousel((int)CAROUSEL_PPR6th);
    }

    public void rotateCarousel(int amt){
        int newPosition = (int) (amt) + position;
        position = newPosition;
        carousel.setTargetPosition(newPosition);
        carousel.setTargetPositionTolerance(5);
        carousel.setPower(1);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateActive = true;
    }

    public void rotateCarousel(int amt, double power){
        int newPosition = (int) (amt) + position;
        position = newPosition;
        carousel.setTargetPosition(newPosition);
        carousel.setTargetPositionTolerance(5);
        carousel.setPower(power);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateActive = true;
    }

    public int getPosition(){
        return position;
    }

    public boolean isRotateActive(){
        return rotateActive;
    }

    public void move(double power){
        carousel.setPower(power);
        position = carousel.getCurrentPosition();
    }

    public void setMode(int mode){
        if (mode == 0)
            carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        else if (mode == 1)
            carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the power of the motor to zero and puts the motor back into manual mode
     */
    public void stop() {
        rotateActive = false;
        move(0);
        setMode(Carousel.MANUAL);
    }

    /**
     * Checks if the carousel has finished its rotation
     * @return true if the rotation was active and has now stopped, false if
     * the rotation wasn't active at all or is still turning
     */
    public boolean isFinished(){
        if (rotateActive && !carousel.isBusy()){
            return true;
        }
        return false;
    }
}
