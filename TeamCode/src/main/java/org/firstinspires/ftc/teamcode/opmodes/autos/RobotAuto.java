package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class RobotAuto extends LinearOpMode {
    public Drivetrain drive;
    public Intake intake;
    public Lift lift;
    public CarouselManipulator duckScorer;
    public Vision vision;
    public Depositor depositor;
    public RRMecanumDrive RRdrive;

    public Subsystem[] subsystems;

    private Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()){
                for (Subsystem subsystem: subsystems){
                    subsystem.update();
                }
                telemetry.update();
            }
        }
    };

    protected Thread updateThread = new Thread(updateRunnable);


    @Override
    public void runOpMode() throws InterruptedException {

    }

    void initialize() {
        drive = new Drivetrain(hardwareMap);
        RRdrive = new RRMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        duckScorer = new CarouselManipulator(hardwareMap);
        vision = new Vision(hardwareMap);
        depositor = new Depositor(hardwareMap);

        subsystems = new Subsystem[] {intake, lift, duckScorer, vision, depositor, RRdrive};
    }



}
