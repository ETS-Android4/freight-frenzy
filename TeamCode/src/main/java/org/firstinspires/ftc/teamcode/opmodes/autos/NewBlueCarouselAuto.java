package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.BlueCarouselTeamElementPipeline;

@Config
@Autonomous
public class NewBlueCarouselAuto extends RobotAuto {

    BlueCarouselTeamElementPipeline.Location elementLocation = BlueCarouselTeamElementPipeline.Location.RIGHT;
    private static int TIME_TO_DUCK_SCORE = 3500;
    private static int TIME_TO_DEPOSIT = 1500;
    private static int EXTEND_TO_ANGLE = 350;
    private static int EXTEND_TO_TOP = 1625; //2530
    private static int EXTEND_TO_MID = 1550; //2400
    private static int EXTEND_TO_BOTTOM = 1510; //2325
    private static int STRAFE_TO_STORAGE = 400; //Formally 1200
    private static int TURN_TO_DUCK_SCORE = 175;
    private static int TURN_IN_STORAGE = 175; //Formally 350
    private static int TURN_TO_COLLECT_DUCK = 250;
    private static int STRAFE_IN_STORAGE = 225;
    private static int STRAFE_TO_DUCK_COLLECT = 800;

//gamer - Randall Delafuente

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        vision.setRobotLocation(Vision.robotLocation.BLUE_CAROUSEL);
        vision.enable();


        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Element Location: ", vision.getElementPipelineBlueCarousel().getLocation());
            telemetry.update();
        }

        waitForStart();


        lift.setExtensionState(Lift.ExtensionState.IDLE);
        //lift.setExtensionState(Lift.ExtensionState.AUTONOMOUS);
        duckScorer.setAllianceSide(CarouselManipulator.Alliance.BLUE);

        updateThread.start();

        elementLocation = vision.getElementPipelineBlueCarousel().getLocation();

        switch (elementLocation){
            case RIGHT:
                /*If the element is in the right most position,
                Angle the lift to the top level of the shipping hub
                Extend to the top level of the shipping hub
                 */
                extendToScore(Lift.AngleState.CAROUSEL_TOP);
                homeLift();

                break;
            case MID:
                /*If the element is in the middle position,
                Angle the lift to the second level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.CAROUSEL_MID);
                homeLift();

                break;
            case LEFT:
                /*If the element is in the left position,
                Angle the lift to the first level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.CAROUSEL_BOTTOM);
                homeLift();

                break;
        }

        sleep(1000);

        //Set carousel manipulator to the scoring state
        duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.SCORING);
        sleep(TIME_TO_DUCK_SCORE);

        //Set the position of the front intake bars to be down
        //Turn on intake
        intake.setIntakeState(Intake.IntakeState.IN);
        duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.STOWED);

        //Wait x seconds
        sleep(2000);

        encoderDrive(0.0, 0.5, 0.0, TURN_TO_COLLECT_DUCK);

        sleep(500);

        encoderDrive(0.0, -0.5, 0.0, TURN_TO_COLLECT_DUCK);

        sleep(500);

        encoderDrive(0.5,0,0, STRAFE_TO_DUCK_COLLECT);
        intake.setIntakeState(Intake.IntakeState.UP);

        sleep(750);

        encoderDrive(0.5,0,0, STRAFE_TO_STORAGE);

        sleep(500);

        //Move to storage unit LOL
        encoderDrive(0.0, -0.5, 0, TURN_TO_DUCK_SCORE);

        extendToScore(Lift.AngleState.CAROUSEL_TOP);

        homeLift();

        encoderDrive(0.0, -0.5, 0, TURN_IN_STORAGE);

        sleep(250);

        encoderDrive(0.5, 0, 0, STRAFE_IN_STORAGE);
    }

    public void homeLift(){
        depositor.setDepositorState(Depositor.depositorState.RESTING);
        lift.setExtensionState(Lift.ExtensionState.HOMING);
        lift.setAnglerState(Lift.AngleState.BOTTOM);
    }

    public void homeLiftFrom(int startPosition){
        //Set the depositor to a resting position
        depositor.setDepositorState(Depositor.depositorState.RESTING);

        //Retract the lift to a "ready to angle state"
        retractToPosition(2000);

        sleep(500);

        //Angle the lift to its home position
        lift.setAnglerState(Lift.AngleState.BOTTOM);

        //Fully retract lift until the magnetic limit switch is hit
        slideRetract();
    }

    public void extendToScore (Lift.AngleState angleState){
        //Extend to "ready to angle position"
        //extendToPosition(EXTEND_TO_ANGLE);
        lift.setExtensionState(Lift.ExtensionState.EXTEND_TO_ANGLE);

        //sleep(1000);

        //Angle slides towards the top level of the shipping hub
        lift.setAnglerState(angleState);
        depositor.setDepositorState(Depositor.depositorState.TOP_ANGLE);

        sleep(250); //Previously 1000

        //Extend slides to the correct level of the shipping hub
        switch (angleState){
            case CAROUSEL_TOP:
                extendToPosition(EXTEND_TO_TOP);
                break;
            case CAROUSEL_MID:
                extendToPosition(EXTEND_TO_MID);
                break;
            case CAROUSEL_BOTTOM:
                extendToPosition(EXTEND_TO_BOTTOM);
                break;
        }

        sleep(1000);

        //Set the depositor to a scoring position
        depositor.setDepositorState(Depositor.depositorState.SCORING);

        //Wait x seconds
        sleep(TIME_TO_DEPOSIT);

    }


    public void extendToPosition(int counts) {
        double startTime = getRuntime();
        //lift.setLiftPower(-1.0); //Formally 0.3
        lift.setExtensionState(Lift.ExtensionState.OUT_AUTO);


        while (opModeIsActive()) {
            if (Math.abs(lift.getLiftPosition()) > counts) {
                break;
            }
            else if (getRuntime() - startTime > 4.0){
                break;
            }
        }

        //lift.setLiftPower(0.0);
        lift.setExtensionState(Lift.ExtensionState.IDLE);
    }

    /*
    public void extendToPosition(int counts) {
        double startTime = getRuntime();
        lift.setLiftPower(-1.0); //Formally 0.3

        while (opModeIsActive()) {
            if (lift.getLiftPosition() > counts) {
                break;
            }
            else if (getRuntime() - startTime > 1000.0){ //Previously 4.0
                break;
            }
        }

        lift.setLiftPower(0.0);
    }

     */




    public void retractToPosition(int endPosition){
        //lift.setLiftPower(1.0);
        lift.setExtensionState(Lift.ExtensionState.IN);

        while (opModeIsActive()) {
            if (Math.abs(lift.getLiftPosition()) < endPosition) {
                break;
            }
        }

        //lift.setLiftPower(0.0);
        lift.setExtensionState(Lift.ExtensionState.IDLE);
    }

    public void slideRetract(){
        lift.setLiftPower(1.0);

        while (opModeIsActive()) {
            if (!lift.getRetractionLimitValue()) {
                break;
            }
        }

        lift.setLiftPower(0.0);
        lift.setLiftMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double x, double y, double turn, int counts) {
        drive.resetEncoders();
        drive.cartesianDrive(x, y, turn);
        while (opModeIsActive()) {

            for (int position : drive.getWheelPositions()) {
                if (Math.abs(position) > counts) {
                    drive.stop();
                    return;
                }
            }
        }
        drive.stop();
    }








}
