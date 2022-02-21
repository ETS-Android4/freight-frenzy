package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class Depositor implements Subsystem{
    private Servo depositorServo;
    private RevColorSensorV3 freightDetector;

    private static boolean freightCheck;
    private static double SCORING_POSITION = 0.21; // 0.1
    private static double RESTING_POSITION = 0.5;
    private static double BALANCE_FOR_TOP = 0.435;
    private static double BALANCE_FOR_MID = 0.35;
    private static double DUCK_ANGLE = 0.435;
    private static double CARGO_CRADLE = 0.26;

    public enum depositorState{
        SCORING,
        RESTING,
        MID_ANGLE,
        TOP_ANGLE,
        DUCK_ANGLE,
        CARGO_CRADLE
    }

    public enum freightType{
        CARGO,
        BOX,
        NONE
    }

    public enum storageState{
        STORED,
        IN,
        NONE
    }

    storageState freightState = storageState.NONE;

    private depositorState state = depositorState.RESTING;

    public Depositor (HardwareMap hardwareMap){
        depositorServo = hardwareMap.get(Servo.class, "depositorServo");
        freightDetector = hardwareMap.get(RevColorSensorV3.class, "freightDetector");
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
                break;
            case MID_ANGLE:
                depositorServo.setPosition(BALANCE_FOR_MID);
                break;
            case DUCK_ANGLE:
                depositorServo.setPosition(DUCK_ANGLE);
                break;
            case CARGO_CRADLE:
                depositorServo.setPosition(CARGO_CRADLE);
                break;
        }
    }

    public void freightCheck (){
            if (freightState == storageState.STORED){

            }
            else if (getDetectionDistanceInches() <= 1.5){
                setStorageState(storageState.IN);
            }
            else if (getDetectionDistanceInches() > 1.5){
                setStorageState(storageState.NONE);
            }

    }

    public void getFreightCheck(){
       if (getDetectionDistanceInches() <= 1.5){
            setStorageState(storageState.IN);
        }

       if (getDetectionDistanceInches() > 1.5){
            setStorageState(storageState.NONE);
        }
    }

    public storageState getStorageState(){
        return freightState;
    }

    public void setStorageState(storageState newState){
        freightState = newState;
    }

    public int getFreightBlue(){
        return freightDetector.blue();
    }

    public double getDepositorPivotPosition () { return depositorServo.getPosition(); }

    public double getDetectionDistanceInches () {
        return freightDetector.getDistance(DistanceUnit.INCH);
    }
    public double getDetectionDistanceCentimeter () {
        return freightDetector.getDistance(DistanceUnit.CM);
    }

    public void setDepositorState (depositorState newState){
        this.state = newState;
    }

    public depositorState getDepositorState(){
        return state;
    }


}
