package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.TeamElementPipeline;

@Autonomous
public class BlueAuto extends RobotAuto {

    TeamElementPipeline.Location elementLocation = TeamElementPipeline.Location.RIGHT;
    private static final int TIME_TO_DUCK_SCORE = 3000;
    private static final int TIME_TO_DEPOSIT = 1000;
    private static final int EXTEND_TO_ANGLE = 100;
    private static final int EXTEND_TO_TOP = 200;
    private static final int EXTEND_TO_MID = 200;
    private static final int EXTEND_TO_BOTTOM = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        vision.enable();

        telemetry.addData("Element Location: ", vision.getElementPipeline().getLocation());


        waitForStart();

        updateThread.start();

        elementLocation = vision.getElementPipeline().getLocation();


        switch (elementLocation){
            case RIGHT:
                /*If the element is in the right most position,
                Angle the lift to the top level of the shipping hub
                Extend to the top level of the shipping hub
                 */
                extendToScore(Lift.AngleState.TOP);
                break;
            case MID:
                /*If the element is in the middle position,
                Angle the lift to the second level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.MID);
                break;
            case LEFT:
                /*If the element is in the left position,
                Angle the lift to the first level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.BOTTOM);
                break;
        }



        //Wait x seconds
        sleep(1000);

        homeLift();

        //Set carousel manipulator to the scoring state
        duckScorer.setManipulatorState(CarouselManipulator.carouselManipulatorState.SCORING);
        sleep(TIME_TO_DUCK_SCORE);

        //Set the position of the front intake bars to be down
        //Turn on intake
        intake.setIntakeState(Intake.IntakeState.IN);

        //Wait x seconds
        sleep(1000);

        extendToScore(Lift.AngleState.TOP);

        homeLift();

        //Move to storage unit LOL






    }

    public void homeLift(){
        //Set the depositor to a resting position
        depositor.setDepositorState(Depositor.depositorState.RESTING);

        //Retract the lift to a "ready to angle state"
        lift.extendToPosition(EXTEND_TO_ANGLE);

        //Angle the lift to its home position
        lift.setAnglerState(Lift.AngleState.BOTTOM);

        //Fully retract lift until the magnetic limit switch is hit
    }

    public void extendToScore (Lift.AngleState angleState){
        //Extend to "ready to angle position"
        lift.extendToPosition(EXTEND_TO_ANGLE);

        //Angle slides towards the top level of the shipping hub
        lift.setAnglerState(angleState);

        //Extend slides to the correct level of the shipping hub
        switch (angleState){
            case TOP:
                lift.extendToPosition(EXTEND_TO_TOP);
                break;
            case MID:
                lift.extendToPosition(EXTEND_TO_MID);
                break;
            case BOTTOM:
                lift.extendToPosition(EXTEND_TO_BOTTOM);
                break;
        }

        //Set the depositor to a scoring position
        depositor.setDepositorState(Depositor.depositorState.SCORING);

        //Wait x seconds
        sleep(TIME_TO_DEPOSIT);

    }





}
