package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Depositor implements Subsystem{
    private Servo depositorServo;
    private static double SCORING_POSITION = 0.0;
    private static double RESTING_POSITION = 0.61;
    private static double BALANCE_FOR_TOP = 0.4;
    private static double BALANCE_FOR_MID = 0.5;

    public enum depositorState{
        SCORING,
        RESTING,
        MID_ANGLE,
        TOP_ANGLE
    }

    private depositorState state = depositorState.RESTING;

    public Depositor (HardwareMap hardwareMap){
        depositorServo = hardwareMap.get(Servo.class, "depositorServo");
    }

    public void update(){
        switch(state){
            case SCORING:
                depositorServo.setPosition(SCORING_POSITION);
                break;
            case RESTING:
                depositorServo.setPosition(RESTING_POSITION);
                break;
            case TOP_ANGLE:
                depositorServo.setPosition(BALANCE_FOR_TOP);
            case MID_ANGLE:
                depositorServo.setPosition(BALANCE_FOR_MID);

        }

    }

    public double getDepositorPivotPosition () { return depositorServo.getPosition(); }

    public void setDepositorState (depositorState newState){
        this.state = newState;
    }

    public depositorState getDepositorState(){
        return state;
    }


}
