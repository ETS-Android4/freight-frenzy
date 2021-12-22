package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;


public class Lift implements Subsystem{

    private DcMotor angler;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private TouchSensor anglerLimit;
    private DigitalChannel retractionLimit;


    private static int TOP_FROM_WAREHOUSE = 475;
    private static int MID_FROM_CAROUSEL = 300;
    private static int TOP_FROM_CAROUSEL = 500;
    private static int MID_FROM_WAREHOUSE = 250;
    private static int BOTTOM_FROM_CAROUSEL = 175;
    private static int TOP_MID_DIFFERENCE = TOP_FROM_WAREHOUSE - MID_FROM_WAREHOUSE;

    private static int BASE_ANGLE;
    private double anglerPower = 0.0;

    public enum AngleState {
        BOTTOM,
        MID,
        TOP,
        IDLE,
        MANUAL_UP,
        MANUAL_DOWN,
        CAROUSEL_TOP,
        CAROUSEL_MID,
        CAROUSEL_BOTTOM
    }

    public enum ExtensionState {
        EXTEND_TO_ANGLE,
        IDLE,
        MANUAL_OUT,
        MANUAL_IN
    }

    private AngleState state = AngleState.BOTTOM;
    private ExtensionState extensionState = ExtensionState.IDLE;



    public Lift(HardwareMap hardwareMap){
        angler = hardwareMap.get(DcMotor.class, "angleAdjuster");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        anglerLimit = hardwareMap.get(TouchSensor.class, "anglerLimit");
        retractionLimit = hardwareMap.get(DigitalChannel.class, "retractionLimit");
        retractionLimit.setMode(DigitalChannel.Mode.INPUT);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

                if (getAngleLimitValue())
                {
                    setAnglerState(Lift.AngleState.IDLE);


                }
                else
                {
                    angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    angler.setPower(0.5);

                }

                break;

            case MID:
                //angler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //angler.setTargetPosition(MID_FROM_CAROUSEL);
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, MID_FROM_WAREHOUSE);
                break;

            case TOP:
                //angler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //angler.setTargetPosition(TOP_FROM_CAROUSEL);
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, TOP_FROM_WAREHOUSE);

                break;

            case IDLE:
                angler.setPower(0.0);
                angler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

            case MANUAL_DOWN:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                angler.setPower(1.0);
                break;

            case MANUAL_UP:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                angler.setPower(-1.0);
                break;

            case CAROUSEL_TOP:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, TOP_FROM_CAROUSEL);
                break;

            case CAROUSEL_MID:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, MID_FROM_CAROUSEL);
                break;

            case CAROUSEL_BOTTOM:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, BOTTOM_FROM_CAROUSEL);
                break;

        }

        switch (extensionState){
            case IDLE:
                setLiftPower(0.0);
                break;
            case EXTEND_TO_ANGLE:
                setExtensionPosition(1.0, 350);
                break;
            case MANUAL_IN:
                setLiftPower(1.0);
                break;
            case MANUAL_OUT:
                setLiftPower(-1.0);
                break;
        }
    }

    public void setLiftPower(double newLiftPower){
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setPower(-newLiftPower);
        liftRight.setPower(newLiftPower);
    }

    public void setLiftMode(DcMotor.RunMode mode){
        liftLeft.setMode(mode);
        liftRight.setMode(mode);
    }

    public void extendToPosition(int counts, double power) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) > counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }

    /*public void extendToPosition(int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(-1.0);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) > counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }
    */

    /*public void retractToPosition(int startPosition, int endPosition){
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(1.0);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) < (startPosition - endPosition)) {
                break;
            }
        }

        setLiftPower(0.0);
    }
    */


    /*public void slideRetract(){
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(1.0);

        while (linearOpMode.opModeIsActive()) {
            if (retractionLimit.getState()) {
                break;
            }
        }

            setLiftPower(0.0);
        }

     */


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

    public void setExtensionState (ExtensionState state) {this.extensionState = state;}

    public AngleState getAnglerState (){
        return state;
    }

    public int getLiftPosition(){
        return liftLeft.getCurrentPosition();
    }

    public int getLiftLeftPosition(){
        return liftLeft.getCurrentPosition();
    }

    public int getLiftRightPosition(){
        return liftRight.getCurrentPosition();
    }

    public boolean getAngleLimitValue(){ return anglerLimit.isPressed(); }

    public boolean getRetractionLimitValue() { return retractionLimit.getState(); }

    public void setAnglerPosition (double power, int encoderCounts){
        angler.setPower(power);
        if (getAnglerPosition() < -encoderCounts) {
            setAnglerState(AngleState.IDLE);
        }
    }

    public void setExtensionPosition (double power, int encoderCounts){
        setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftPower(power);
        if (getLiftPosition() > encoderCounts){
            setExtensionState(ExtensionState.IDLE);
        }
    }
}
