package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Lift implements Subsystem{

    private DcMotor angler;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private TouchSensor anglerLimit;
    private DigitalChannel retractionLimit;
    private DigitalChannel topMagnetStop;

    public static int AUTO_TOP_FOR_WAREHOUSE = 555;
    public static int AUTO_MID_FOR_WAREHOUSE = 350;
    public static int AUTO_BOTTOM_FOR_WAREHOUSE = 150;

    public static int FIRST_TOP_EXTEND = 1325;
    public static int TOP_EXTEND = 860; //1295 48000
    public static int MID_EXTEND = 1195; //1275 47000
    public static int BOTTOM_EXTEND = 1200;

    public static double AUTO_EXTEND_POWER = -0.85;

    public static int TOP_FROM_WAREHOUSE = 600; //Previously 460, Through-Bore: 273, Regular Encoder: 500, Auto: 550
    public static int MID_FROM_WAREHOUSE = 300; // Through-Bore: 150, Regular Encoder: 250

    public static int TOP_FROM_CAROUSEL = 480;
    public static int MID_FROM_CAROUSEL = 275;
    public static int BOTTOM_FROM_CAROUSEL = 175;
    public static int RETRACT_TO_ANGLE = 600;
    public static int DUCK_SCORE = 300;
    public static int CAP_SCORE = 635; // 400
    public static int CAP_COLLECT = 25;

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
        CAROUSEL_BOTTOM,
        ADJUST_DOWN,
        DUCK,
        CAP_SCORE,
        CAP_COLLECT,
        AUTO_TOP,
        AUTO_MID,
        AUTO_BOTTOM,
        TOP_MAG
    }

    public enum ExtensionState {
        EXTEND_TO_ANGLE,
        IDLE,
        OUT,
        IN,
        HOMING,
        RETRACT_TO_ANGLE,
        OUT_AUTO,
        CAP_COLLECT,
        EXTEND_TO_TOP,
        EXTEND_TO_MID,
        EXTEND_TO_BOTTOM,
        FIRST_TOP_EXTEND,
        EXTEND_TO_ANGLE_TOP
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
        topMagnetStop = hardwareMap.get(DigitalChannel.class, "topMagnetStop");
        topMagnetStop.setMode(DigitalChannel.Mode.INPUT);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            case TOP_MAG:
                angler.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                runToMagnet(-1.0);
                break;

            case DUCK:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, DUCK_SCORE);

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

            case ADJUST_DOWN:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPositionDown(1.0, 10);
                break;

            case CAP_SCORE:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-0.75, CAP_SCORE);
                break;

            case CAP_COLLECT:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-0.25, CAP_COLLECT);
                break;

            case AUTO_TOP:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, AUTO_TOP_FOR_WAREHOUSE);
                break;

            case AUTO_MID:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, AUTO_MID_FOR_WAREHOUSE);
                break;

            case AUTO_BOTTOM:
                angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setAnglerPosition(-1.0, AUTO_BOTTOM_FOR_WAREHOUSE);
                break;
        }

        switch (extensionState){
            case IDLE:
                setLiftPower(0.0);
                setLiftMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case EXTEND_TO_ANGLE:
                extendToPosition(-1.0, 350); //350 23000
                //setExtensionState(ExtensionState.IDLE);
                break;

            case EXTEND_TO_ANGLE_TOP:
                extendToPosition(-1.0, 250); //350 23000
                //setExtensionState(ExtensionState.IDLE);
                break;

            case IN:
                setLiftPower(1.0);
                break;
            case OUT:
                //setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                setLiftPower(-1.0);
                break;
            case HOMING:
                retractToPosition(1.0, RETRACT_TO_ANGLE);
                setAnglerState(AngleState.BOTTOM);
                fullyRetract();
                //setExtensionState(ExtensionState.IDLE);
                break;
            case OUT_AUTO:
                setLiftPower(-0.99);
                break;
            case CAP_COLLECT:
                extendToPosition(-1.0, 400); //400 25000
                break;
            case EXTEND_TO_TOP:
                extendToPosition(AUTO_EXTEND_POWER, TOP_EXTEND);
                break;

            case EXTEND_TO_MID:
                extendToPosition(AUTO_EXTEND_POWER, MID_EXTEND);
                break;

            case EXTEND_TO_BOTTOM:
                extendToPosition(AUTO_EXTEND_POWER, BOTTOM_EXTEND);
                break;

            case FIRST_TOP_EXTEND:
                extendToPosition(AUTO_EXTEND_POWER, FIRST_TOP_EXTEND);
                break;
        }
    }

    public void setLiftPower(double newLiftPower){
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(-newLiftPower);
        liftRight.setPower(newLiftPower);
    }

    public void setLiftMode(DcMotor.RunMode mode){
        liftLeft.setMode(mode);
        liftRight.setMode(mode);
    }

    /*public void extendOut(int counts, double power) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) > counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }
    */


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

    public ExtensionState getExtensionState () {return extensionState;}

    public int getLiftPosition(){

        //return -liftLeft.getCurrentPosition();
        return liftLeft.getCurrentPosition();
    }

    public boolean getTopMagnetValue() {return topMagnetStop.getState();}

    public int getLiftLeftPosition(){
        return -liftLeft.getCurrentPosition();
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

    public void runToMagnet (double power){
        angler.setPower(power);
        if (getTopMagnetValue() != true){
            setAnglerState(AngleState.IDLE);
        }
    }

    public void setAnglerPositionDown (double power, int encoderCounts){
        angler.setPower(power);
        int startingAnglePosition = getAnglerPosition();

        if (getAnglerPosition() < startingAnglePosition - (-encoderCounts)){
            setAnglerState(AngleState.IDLE);
        }
    }

    public void extendToPosition(double power, int encoderCounts){
        setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftPower(power);
        if ((getLiftLeftPosition()/37) > encoderCounts){ //Swapped from getLiftPosition to getLiftRightPosition
            setExtensionState(ExtensionState.IDLE);
        }
    }

    public void retractToPosition (double power, int encoderCounts){
        setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftPower(power);
        int startingLiftPosition = getLiftPosition();
        if (getLiftPosition() < startingLiftPosition - encoderCounts){
            setExtensionState(ExtensionState.IDLE);
        }
    }

    public void fullyRetract(){
        setLiftPower(1.0);
        if (!getRetractionLimitValue()){
            setExtensionState(ExtensionState.IDLE);
        }
    }
}
