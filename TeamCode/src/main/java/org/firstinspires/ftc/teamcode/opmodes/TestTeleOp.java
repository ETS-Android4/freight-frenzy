package org.firstinspires.ftc.teamcode.opmodes;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Test TeleOp")
public class TestTeleOp extends RobotOpMode{

    @Override
    public void init() {
        super.init();
        intake.setIntakeState(Intake.IntakeState.OFF);
        drive.brakeMode(true);
    }

    @Override
    public void loop() {
        super.loop();

        drive.cartesianDrive(gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_stick_y);

        if (epicGamer1.RIGHT_BUMPER.pressed()){
            if (intake.getIntakeState() != Intake.IntakeState.IN){
                intake.setIntakeState(Intake.IntakeState.IN);
            }
            else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }
        }

        if (epicGamer1.LEFT_BUMPER.pressed()){
            if (intake.getIntakeState() != Intake.IntakeState.OUT){
                intake.setIntakeState(Intake.IntakeState.OUT);
            }
            else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }
        }
    }
}
