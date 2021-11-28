package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Lift implements Subsystem{

    private DcMotor angler;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private TouchSensor anglerLimit;


    private static int TOP_FROM_CAROUSEL;
    private static int MID_FROM_CAROUSEL;
    private static int TOP_MID_DIFFERENCE = TOP_FROM_CAROUSEL - MID_FROM_CAROUSEL;
    private static int BASE_ANGLE;
    private double anglerPower = 0.0;

    /*
    public enum OldAngleState {
        BOTTOM,
        MID,
        TOP
    }
    */



    public enum AngleState {
        BOTTOM,
        MID,
        TOP
    }

    private AngleState state = AngleState.BOTTOM;
    //private OldAngleState oldState = OldAngleState.BOTTOM;

    public Lift(HardwareMap hardwareMap){
        angler = hardwareMap.get(DcMotor.class, "angleAdjuster");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        anglerLimit = hardwareMap.get(TouchSensor.class, "anglerLimit");

        //angler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAnglerPower(double newLiftPower){
        angler.setPower(newLiftPower);
    }

    public double getAnglerPower(){
        return angler.getPower();
    }
    public int getAnglerPosition(){
        return angler.getCurrentPosition();
    }

    public void update(){
        switch (state){
            case BOTTOM:
                //angler.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                angler.setPower(-0.5);
                if (anglerLimit.isPressed()){
                    angler.setPower(0.0);
                    angler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                break;

            case MID:
                angler.setTargetPosition(MID_FROM_CAROUSEL);
                break;

            case TOP:
                angler.setTargetPosition(TOP_FROM_CAROUSEL);
                break;
        }
    }

    public void setLiftPower(double newLiftPower){
        liftLeft.setPower(-newLiftPower);
        liftRight.setPower(newLiftPower);
    }

    public void setLiftMode(DcMotor.RunMode mode){
        liftLeft.setMode(mode);
        liftRight.setMode(mode);
    }

    public void extendToPosition(int newPosition){
        setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setTargetPosition(newPosition);
        liftRight.setTargetPosition(-newPosition);
    }

    public void setLeftLiftPower(double newLeftLiftPower){
        liftLeft.setPower(newLeftLiftPower);
    }

    public void setRightLiftPower(double newRightLiftPower){
        liftRight.setPower(newRightLiftPower);
    }

    public double getLiftPower(){
        return liftRight.getPower();
    }

    public void setAnglerState (AngleState state) {
        this.state = state;
    }

    public AngleState getAnglerState (){
        return state;
    }

}
