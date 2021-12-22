package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autos.RobotAuto;
import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends RobotOpMode {


    @Override
    public void init() {
        super.init();
        intake.setIntakeState(Intake.IntakeState.OFF);
        drive.brakeMode(true);
        depositor.setDepositorState(Depositor.depositorState.RESTING);
        duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.STOWED);
        duckScorer.setAllianceSide(CarouselManipulator.Alliance.BLUE);
    }

    public enum AngleModeBlue{
        MANUAL,
        AUTO
    }

    private BlueTeleOp.AngleModeBlue anglerSwitch = BlueTeleOp.AngleModeBlue.AUTO;

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

        if (epicGamer1.X.pressed()){
            depositor.setDepositorState(Depositor.depositorState.SCORING);
        }
        if(epicGamer2.X.pressed()){
            depositor.setDepositorState(Depositor.depositorState.RESTING);
        }

        if (epicGamer1.DPAD_UP.pressed()){
            intake.setIntakeState(Intake.IntakeState.UP);
        }


        if (gamepad2.right_trigger != 0){
            lift.setLiftPower(1.0);
        }

        else if (gamepad2.left_trigger != 0 && lift.getRetractionLimitValue()){
            lift.setLiftPower(-1.0);
            depositor.setDepositorState(Depositor.depositorState.RESTING);
        }

        else {
            lift.setLiftPower(0.0);
        }


        if (epicGamer1.DPAD_LEFT.state){
            duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.REST);
        }
        if (epicGamer1.DPAD_RIGHT.state){
            duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.SCORING);
        }
        if (epicGamer1.DPAD_DOWN.state){
            duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.STOWED);
        }

        if (epicGamer2.DPAD_RIGHT.pressed()){
            if (anglerSwitch != BlueTeleOp.AngleModeBlue.AUTO) {
                anglerSwitch = BlueTeleOp.AngleModeBlue.AUTO;
            }
            else {
                anglerSwitch = BlueTeleOp.AngleModeBlue.MANUAL;
            }
        }

        switch (anglerSwitch){
            case AUTO:
                if (epicGamer2.A.pressed()){
                    lift.setAnglerState(Lift.AngleState.BOTTOM);
                    depositor.setDepositorState(Depositor.depositorState.RESTING);
                }
                if (epicGamer2.B.pressed()){
                    lift.setAnglerState(Lift.AngleState.MID);
                    depositor.setDepositorState(Depositor.depositorState.MID_ANGLE);
                }
                if (epicGamer2.Y.pressed()) {
                    lift.setAnglerState(Lift.AngleState.TOP);
                    depositor.setDepositorState(Depositor.depositorState.TOP_ANGLE);
                }
                break;
            case MANUAL:
                if (epicGamer2.DPAD_UP.state){
                    lift.setAnglerState(Lift.AngleState.MANUAL_UP);
                }
                else if (epicGamer2.DPAD_DOWN.state){
                    lift.setAnglerState(Lift.AngleState.MANUAL_DOWN);
                }
                else {
                    lift.setAnglerState(Lift.AngleState.IDLE);
                }
                break;
        }

        telemetry.addData("Angler Position: ", lift.getAnglerPosition());
        telemetry.addData("liftLeft Position: ", lift.getLiftLeftPosition());
        telemetry.addData("liftRight Position: ", lift.getLiftRightPosition());
        telemetry.addData("Limit pressed = ", lift.getAngleLimitValue());
        telemetry.addData("Angle State: ", lift.getAnglerState());
        telemetry.addData("Angler Power: ", lift.getAnglerPower());
        telemetry.addData("Depositor Pivot Position: ", depositor.getDepositorPivotPosition());
        telemetry.addData("Retraction Limit: ", !lift.getRetractionLimitValue());

    }
}
