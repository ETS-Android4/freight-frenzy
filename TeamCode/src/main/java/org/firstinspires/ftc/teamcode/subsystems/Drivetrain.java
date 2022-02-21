package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Drivetrain implements Subsystem {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1.9, 0, 0.8);


    private DcMotor leftFront, leftBack, rightFront, rightBack;
    BNO055IMU imu;


    private double[] powers = new double[4];


    private OpMode opMode;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightFront");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    /*
    public void turnToAngle (double turn, double angle){
        double output;


    }

     */

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
        rightBack.setPower(-powers[3]);
    }


}
