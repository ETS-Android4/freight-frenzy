package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake implements Subsystem {

    private DcMotor intakeMotor;

    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    private IntakeState state = IntakeState.OFF;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }


    public void update(){
        switch(state){
            case IN:
                intakeMotor.setPower(1.0);
                break;
            case OUT:
                intakeMotor.setPower(-1.0);
                break;
            case OFF:
                intakeMotor.setPower(0.0);
        }

    }

    public IntakeState getIntakeState(){
        return state;
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }


}
