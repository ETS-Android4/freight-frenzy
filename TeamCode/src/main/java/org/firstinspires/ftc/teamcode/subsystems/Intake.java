package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;


public class Intake implements Subsystem {

    private DcMotor intakeMotor;
    private Servo intakePivot;
    private static double INTAKE_DOWN = 0.1;
    private static double INTAKE_UP = 0.5;

    public enum IntakeState {
        IN,
        OUT,
        OFF,
        UP,
        DUCK
    }

    private IntakeState state = IntakeState.UP;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

    }

    public void update(){
        switch(state){
            case IN:
                intakePivot.setPosition(INTAKE_DOWN);
                intakeMotor.setPower(1.0);
                break;
            case OUT:
                intakeMotor.setPower(-1.0);
                break;
            case OFF:
                intakeMotor.setPower(0.0);
                break;
            case UP:
                intakeMotor.setPower(0.0);
                intakePivot.setPosition(INTAKE_UP);
                break;
            case DUCK:
                intakeMotor.setPower(0.5);
                break;
        }

    }

    public IntakeState getIntakeState(){
        return state;
    }

    public int getIntakePosition(){
        return intakeMotor.getCurrentPosition();
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }


}
