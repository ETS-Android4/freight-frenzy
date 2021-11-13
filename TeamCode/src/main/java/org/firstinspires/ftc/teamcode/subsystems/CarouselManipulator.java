package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselManipulator implements Subsystem {

    private Servo turretServo;
    private CRServo carouselSpinner;
    public static double carousel_SCORING;
    public static double carousel_REST;
    public static double carousel_STOW;



    public enum carouselManipulatorState{
        SCORING,
        REST,
        STOWED
    }

    public carouselManipulatorState manipulatorState = carouselManipulatorState.STOWED;


    public CarouselManipulator(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        carouselSpinner = hardwareMap.get(CRServo.class, "spinner");
    }




    public void update(){
        switch (manipulatorState){
            case SCORING:
                turretServo.setPosition(carousel_SCORING);
                carouselSpinner.setPower(1.0);
                break;
            case REST:
                turretServo.setPosition(carousel_REST);
                carouselSpinner.setPower(0.0);
                break;

            case STOWED:
                turretServo.setPosition(carousel_STOW);
                carouselSpinner.setPower(0.0);
                break;
        }


    }

    public carouselManipulatorState getIntakeState(){
        return manipulatorState;
    }

    public void setManipulatorState(CarouselManipulator.carouselManipulatorState state) {
        this.manipulatorState = state;
    }

}
