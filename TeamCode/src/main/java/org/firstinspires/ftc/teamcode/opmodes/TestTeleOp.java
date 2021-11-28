package org.firstinspires.ftc.teamcode.opmodes;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


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
        if (gamepad2.right_trigger != 0){
            lift.setLiftPower(1.0);
        }
        else if (gamepad2.left_trigger != 0){
            lift.setLiftPower(-1.0);
        }
        else {
            lift.setLiftPower(0.0);
        }

        if (epicGamer1.DPAD_LEFT.state){
            duckScorer.setManipulatorState(CarouselManipulator.carouselManipulatorState.REST);
        }
        if (epicGamer1.DPAD_RIGHT.state){
            duckScorer.setManipulatorState(CarouselManipulator.carouselManipulatorState.SCORING);
        }
        if (epicGamer1.DPAD_DOWN.state){
            duckScorer.setManipulatorState(CarouselManipulator.carouselManipulatorState.STOWED);
        }

        if (epicGamer2.A.state){
            lift.setAnglerState(Lift.AngleState.BOTTOM);
        }

        if (epicGamer2.B.state){
            lift.setAnglerState(Lift.AngleState.MID);
        }

        if (epicGamer2.Y.state){
            lift.setAnglerState(Lift.AngleState.TOP);
        }


        if (epicGamer2.DPAD_UP.state){
            lift.setAnglerPower(1.0);
        }
        else if (epicGamer2.DPAD_DOWN.state){
            lift.setAnglerPower(-1.0);
        }
        else {
            lift.setAnglerPower(0.0);
        }

        telemetry.addData("Angler Position: ", lift.getAnglerPosition());
    }
}
