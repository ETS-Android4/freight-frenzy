package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Depositor implements Subsystem{
    private Servo depositorServo;
    private static double SCORING_POSITION = 0.25;
    private static double RESTING_POSITION = 0.75;

    public enum depositorState{
        SCORING,
        RESTING
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
        }

    }

    public void setDepositorState (depositorState newState){
        this.state = newState;
    }

    public depositorState getDepositorState(){
        return state;
    }


}
