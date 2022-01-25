package org.firstinspires.ftc.teamcode.opmodes.autos;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.BlueCarouselTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.BlueWarehouseTeamElementPipeline;

public class BlueWarehouseAuto extends RobotAuto {

    BlueWarehouseTeamElementPipeline.Location elementLocation = BlueWarehouseTeamElementPipeline.Location.RIGHT;

    private int INITIAL_DRIVE_BACK = 200;
    private int DRIVE_TO_WAREHOUSE = 800;
    private int DRIVE_OUT_OF_WAREHOUSE = 800;


    private int TIME_TO_DEPOSIT = 1000;
    private int EXTEND_TO_TOP = 1000;
    private int EXTEND_TO_MID = 1000;
    private int EXTEND_TO_BOTTOM = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        vision.setRobotLocation(Vision.robotLocation.BLUE_WAREHOUSE);
        vision.enable();

        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Element Location: ", vision.getElementPipelineBlueWarehouse().getLocation());
            telemetry.update();
        }


        waitForStart();

        lift.setExtensionState(Lift.ExtensionState.IDLE);

        updateThread.start();

        elementLocation = vision.getElementPipelineBlueWarehouse().getLocation();

        //Drive backwards slightly
        drive.encoderDrive(0.0, 0.0, -1.0, INITIAL_DRIVE_BACK);

        //Turn towards shipping hub


        switch (elementLocation){
            case RIGHT:
                /*If the element is in the right most position,
                Angle the lift to the top level of the shipping hub
                Extend to the top level of the shipping hub
                 */
                extendToScore(Lift.AngleState.TOP);
                homeLift();

                break;
            case MID:
                /*If the element is in the middle position,
                Angle the lift to the second level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.MID);
                homeLift();

                break;
            case LEFT:
                /*If the element is in the left position,
                Angle the lift to the first level of the shipping hub
                Extend to the second level of the shipping hub
                 */
                extendToScore(Lift.AngleState.BOTTOM);
                homeLift();

                break;
        }

        //Turn to be parallel with wall


        //Strafe until side mounted sensor reach an average of a certain value


        //Drive straight into the warehouse
        drive.encoderDrive(0.0, 0.0, 1.0, DRIVE_TO_WAREHOUSE);

        //Turn the intake on and continue to drive slowly into the warehouse until a freight is detected
        intake.setIntakeState(Intake.IntakeState.IN);


        //Set the intake to the out state
        intake.setIntakeState(Intake.IntakeState.OUT);

        //Drive backwards to exit the warehouse
        drive.encoderDrive(0.0, 0.0, -1.0, DRIVE_OUT_OF_WAREHOUSE);

        //Turn towards the shipping hub



        //Extend the lift to score in the top level of the hub
        extendToScore(Lift.AngleState.TOP);



        //Repeat above commands until a time check indicates that the robot needs to park.






    }

    public void homeLift(){
        depositor.setDepositorState(Depositor.depositorState.RESTING);
        lift.setExtensionState(Lift.ExtensionState.HOMING);
        lift.setAnglerState(Lift.AngleState.BOTTOM);
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
            case TOP:
                extendToPosition(EXTEND_TO_TOP);
                break;
            case MID:
                extendToPosition(EXTEND_TO_MID);
                break;
            case BOTTOM:
                extendToPosition(EXTEND_TO_BOTTOM);
                break;
        }

        sleep(1000);

        //Set the depositor to a scoring position
        depositor.setDepositorState(Depositor.depositorState.SCORING);

        //Wait x seconds
        sleep(TIME_TO_DEPOSIT);

    }


}
