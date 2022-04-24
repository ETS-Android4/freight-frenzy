package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselManipulator implements Subsystem {

    private Servo turretServo;
    private CRServo carouselSpinner;
    public static double CAROUSEL_SCORING_BLUE = 0.025;
    public static double CAROUSEL_REST_BLUE = 0.25;

    public static double CAROUSEL_SCORING_RED = 1.0;
    public static double CAROUSEL_REST_RED = 0.9;

    public static double CAROUSEL_STOW = 0.41;

    public enum CarouselManipulatorState {
        SCORING,
        REST,
        STOWED,
        AUTO_SCORING
    }
    public enum Alliance {
        BLUE,
        RED
    }

    public CarouselManipulatorState manipulatorState = CarouselManipulatorState.STOWED;
    public Alliance fieldSide = Alliance.BLUE;


    public CarouselManipulator(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        carouselSpinner = hardwareMap.get(CRServo.class, "duckSpinner");
    }




    public void update(){

        switch (fieldSide){
            case RED:
                switch (manipulatorState){
                    case SCORING:
                        turretServo.setPosition(CAROUSEL_SCORING_BLUE);
                        carouselSpinner.setPower(-1.0);
                        break;

                    case REST:
                        turretServo.setPosition(CAROUSEL_SCORING_BLUE);
                        carouselSpinner.setPower(0.0);
                        break;

                    case STOWED:
                        turretServo.setPosition(CAROUSEL_STOW);
                        carouselSpinner.setPower(0.0);
                        break;

                    case AUTO_SCORING:
                        turretServo.setPosition(CAROUSEL_SCORING_BLUE);
                        carouselSpinner.setPower(-0.75);
                        break;
                }
                break;
            case BLUE:
                switch (manipulatorState){
                    case SCORING:
                        turretServo.setPosition(CAROUSEL_SCORING_RED);
                        carouselSpinner.setPower(1.0);
                        break;

                    case REST:
                        turretServo.setPosition(CAROUSEL_SCORING_RED);
                        carouselSpinner.setPower(0.0);
                        break;

                    case STOWED:
                        turretServo.setPosition(CAROUSEL_STOW);
                        carouselSpinner.setPower(0.0);
                        break;

                    case AUTO_SCORING:
                        turretServo.setPosition(CAROUSEL_SCORING_RED);
                        carouselSpinner.setPower(0.75);
                }
                break;


        }



    }

    public CarouselManipulatorState getIntakeState(){
        return manipulatorState;
    }

    public void setManipulatorState(CarouselManipulatorState state) {
        this.manipulatorState = state;
    }

    public void setAllianceSide (Alliance fieldSide){
        this.fieldSide = fieldSide;
    }

}
