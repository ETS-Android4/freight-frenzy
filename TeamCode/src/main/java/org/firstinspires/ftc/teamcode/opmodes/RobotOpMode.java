package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


public abstract class RobotOpMode extends OpMode {
    public Drivetrain drive;
    public Intake intake;
    public Lift lift;

    public Subsystem[] subsystems;
    public EnhancedGamepad epicGamer1 = new EnhancedGamepad(gamepad1);
    public EnhancedGamepad epicGamer2 = new EnhancedGamepad(gamepad2);

    @Override
    public void init(){
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        epicGamer1 = new EnhancedGamepad(gamepad1);
        epicGamer2 = new EnhancedGamepad(gamepad2);

        subsystems = new Subsystem[] {drive, intake};
    }

    @Override
    public void loop(){
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
        epicGamer1.update();
        epicGamer2.update();
    }

}
