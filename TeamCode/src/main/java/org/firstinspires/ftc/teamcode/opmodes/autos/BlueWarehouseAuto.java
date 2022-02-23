package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.CarouselManipulator;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.BlueWarehouseTeamElementPipeline;

@Autonomous
public class BlueWarehouseAuto extends RobotAuto {

    BlueWarehouseTeamElementPipeline.Location elementLocation = BlueWarehouseTeamElementPipeline.Location.RIGHT;


    public static final Pose2d startPose = new Pose2d(6, 63);
    public static final Pose2d SCORE_POSE = new Pose2d(-6, 60, Math.toRadians(320));
    public static final Pose2d GAP_OUTER_POSE = new Pose2d(10, 62, Math.toRadians(255));
    //public static final Pose2d GAP_INNER_POSE = new Pose2d(36, 60, Math.toRadians(-75));
    public static final Pose2d GAP_INNER_POSE = new Pose2d(10, 25, Math.toRadians(255));
    public static final Pose2d PARK_POSE = new Pose2d(36, 36, Math.toRadians(-90));

    public Pose2d lastPose = startPose;
    public double lastHeading = 0;

    private static int TURN_TO_WALL = -59;

    private static int EXTEND_TO_ANGLE = 350;
    private static int EXTEND_TO_TOP = 1150; //2530
    private static int EXTEND_TO_MID = 1550; //2400
    private static int EXTEND_TO_BOTTOM = 1510; //2325
    private static int TIME_TO_DEPOSIT = 1500;

    Trajectory toFirstScore;
    Trajectory toWarehouse;
    Trajectory toCollect;
    Trajectory toScore;
    Trajectory toSafePark;
    Trajectory toFirstWarehouse;
    Trajectory toGapExit;
    Trajectory toGapEntrance;
    Trajectory toOuterField;

    private enum CyclingState{
        COLLECTING,
        TRAVELLING,
        DELIVERING,
        NULL
    }

    CyclingState cycleState = CyclingState.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        vision.setRobotLocation(Vision.robotLocation.BLUE_WAREHOUSE);
        vision.enable();

        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Element Location: ", vision.getElementPipelineBlueWarehouse().getLocation());
            telemetry.update();
        }

        RRdrive.setPoseEstimate(startPose);

        duckScorer.setManipulatorState(CarouselManipulator.CarouselManipulatorState.STOWED);

        waitForStart();

        lift.setExtensionState(Lift.ExtensionState.IDLE);

        updateThread.start();

        buildTrajectories();

        elementLocation = vision.getElementPipelineBlueWarehouse().getLocation();

        //Drive backwards slightly and turn towards shipping hub

        //cycleState = CyclingState.DELIVERING;
        RRdrive.followTrajectory(toFirstScore);

        switch (elementLocation){

            case RIGHT:
                /*If the element is in the right most position,
                Angle the lift to the top level of the shipping hub
                Extend to the top level of the shipping hub */

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


        sleep(2000);

        //RRdrive.turn(Math.toRadians(TURN_TO_WALL));

        //sleep(2000);

        lastPose = RRdrive.getPoseEstimate();

        sleep(500);
        //Turn to be parallel with wall
        //cycleState = CyclingState.TRAVELLING;
        RRdrive.followTrajectory(toGapEntrance);

        sleep(2000);

        RRdrive.followTrajectory(toWarehouse);


        //Strafe until both side mounted sensors reach a certain value

        /*
        while(cycleState != CyclingState.NULL && !isStopRequested() && opModeIsActive()){
            depositor.freightCheck();

            switch (cycleState){
                case TRAVELLING:
                    RRdrive.followTrajectory(toGapExit);
                    RRdrive.followTrajectory(toOuterField);
                    RRdrive.followTrajectory(toScore);
                    cycleState = CyclingState.DELIVERING;
                    break;

                case DELIVERING:
                    extendToScore(Lift.AngleState.TOP);
                    homeLift();
                    RRdrive.followTrajectory(toGapEntrance);
                    RRdrive.followTrajectory(toWarehouse);
                    cycleState = CyclingState.COLLECTING;
                    break;

                case COLLECTING:
                    if(getRuntime() > 24){
                        RRdrive.breakFollowing();
                        getRobotPosition();
                        cycleState = CyclingState.NULL;
                    }

                    else if (depositor.getStorageState() != Depositor.storageState.IN){
                        intake.setIntakeState(Intake.IntakeState.IN);
                        RRdrive.followTrajectory(toCollect);
                    }

                    else {
                        intake.setIntakeState(Intake.IntakeState.OUT);
                        RRdrive.breakFollowing();
                        getRobotPosition();
                        cycleState = CyclingState.TRAVELLING;
                    }
                    break;

            }
        }

        RRdrive.followTrajectory(toSafePark);

         */




        //Drive straight into the warehouse


        //Turn the intake on and continue to drive slowly into the warehouse until a freight is detected



        //Set the intake to the out state


        //Drive backwards to exit the warehouse


        //Turn towards the shipping hub



        //Extend the lift to score in the top level of the hub




        //Repeat above commands until a time check indicates that the robot needs to park.



    }

    void buildTrajectories(){
        toFirstScore = RRdrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(SCORE_POSE)
                .build();


        /*
        toGapEntrance = RRdrive.trajectoryBuilder(toFirstScore.end().plus(new Pose2d(0, 0, Math.toRadians(TURN_TO_WALL)))) //toScore.end()
                .lineToConstantHeading(GAP_OUTER_POSE)
                .build();

         */


        toGapEntrance = RRdrive.trajectoryBuilder(lastPose) //toScore.end()
                .lineToLinearHeading(GAP_OUTER_POSE)
                .build();


        toWarehouse = RRdrive.trajectoryBuilder(toGapEntrance.end())
                .lineToLinearHeading(GAP_INNER_POSE)
                .build();

        /*
        toCollect = RRdrive.trajectoryBuilder(toWarehouse.end())
                .forward(12)
                .build();

        toGapExit = RRdrive.trajectoryBuilder(lastPose)
                .splineToLinearHeading(GAP_INNER_POSE, lastHeading)
                .build();

        toOuterField = RRdrive.trajectoryBuilder(toGapExit.end())
                .lineToLinearHeading(GAP_OUTER_POSE)
                .addSpatialMarker(new Vector2d(20, 60), () -> {
                    intake.setIntakeState(Intake.IntakeState.OFF);
                })
                .build();

        toScore = RRdrive.trajectoryBuilder(toOuterField.end())
                .lineToLinearHeading(SCORE_POSE)
                .build();

        toSafePark = RRdrive.trajectoryBuilder(lastPose)
                .splineToLinearHeading(PARK_POSE, lastHeading)
                .build();

         */

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

    public void setCyclingState (CyclingState newState) {
        cycleState = newState;
    }

    public void getRobotPosition() {
        lastPose = RRdrive.getPoseEstimate();
        lastHeading = RRdrive.getExternalHeading();
    }


}
