package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Drivetrain implements Subsystem {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();


    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private double[] powers = new double[4];


    private OpMode opMode;

    public Drivetrain(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");
    }

    public void setMotorPowers(double v, double v1, double v2, double v3){
        powers[0] = v;
        powers[1] = v1;
        powers[2] = v2;
        powers[3] = v3;

    }

    public void resetEncoders(){
        for (DcMotor motor : Arrays.asList(leftFront, leftBack, rightBack, rightFront)){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void brakeMode(boolean on){
        if (on){
            for (DcMotor motor : Arrays.asList(leftFront, leftBack, rightBack, rightFront)){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else {
            for (DcMotor motor : Arrays.asList(leftFront, leftBack, rightBack, rightFront)){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    public void cartesianDrive(double x, double y, double turn) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;

        setMotorPowers(v2, v3, v4, v1);
    }

    public void encoderDrive(double x, double y, double turn, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        resetEncoders();
        cartesianDrive(x, y, turn);
        while (linearOpMode.opModeIsActive()) {

            for (int position : getWheelPositions()) {
                if (Math.abs(position) > counts) {
                    stop();
                    return;
                }
            }
        }
        stop();
    }

    public void singleEncoderDrive(double x, double y, double turn, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        resetEncoders();
        double start = System.currentTimeMillis();
        cartesianDrive(x, y, turn);

        while (linearOpMode.opModeIsActive()) {
            if (getWheelPositions().get(0) > counts) {
                break;
            }

            if (System.currentTimeMillis() - start >= 5000) {
                break;
            }
        }
        stop();
    }




    public List<Integer> getWheelPositions() {
        List<Integer> positions = new ArrayList<>();
        positions.add(leftFront.getCurrentPosition());
        positions.add(rightFront.getCurrentPosition());
        positions.add(leftBack.getCurrentPosition());
        positions.add(rightBack.getCurrentPosition());

        return positions;
    }

    public void stop() {
        setMotorPowers(0 , 0, 0, 0);
    }

    public void update() {
        leftFront.setPower(powers[0]);
        rightFront.setPower(powers[1]);
        leftBack.setPower(powers[2]);
        rightBack.setPower(powers[3]);
    }


}
