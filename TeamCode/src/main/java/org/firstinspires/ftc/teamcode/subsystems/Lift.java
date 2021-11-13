package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Lift implements Subsystem{

    private DcMotor angler;
    private DcMotor liftLeft;
    private DcMotor liftRight;


    private int TOP_FROM_WAREHOUSE;
    private int MID_FROM_WAREHOUSE;
    private int TOP_MID_DIFFERENCE = TOP_FROM_WAREHOUSE - MID_FROM_WAREHOUSE;
    private int BASE_ANGLE;
    private double anglerPower = 0.0;

    public enum OldAngleState {
        BOTTOM,
        MID,
        TOP
    }

    public enum AngleState {
        BOTTOM,
        MID,
        TOP
    }

    private AngleState state = AngleState.BOTTOM;
    private OldAngleState oldState = OldAngleState.BOTTOM;

    public Lift(HardwareMap hardwareMap){
        angler = hardwareMap.get(DcMotor.class, "angleAdjuster");
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

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

                break;

            case MID:
                angler.setTargetPosition(MID_FROM_WAREHOUSE);

                break;

            case TOP:
                angler.setTargetPosition(TOP_FROM_WAREHOUSE);
                break;
        }
    }

    public void setLiftPower(double newLiftPower){
        liftLeft.setPower(-newLiftPower);
        liftRight.setPower(newLiftPower);
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
